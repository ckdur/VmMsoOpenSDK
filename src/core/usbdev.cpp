#include "usbdev.h"

#include <libusb.h>

#include <cstdlib>
#include <cstring>

#include "log.h"
#include "usbmng.h"

namespace vmopen {

static const unsigned kUsbTimeoutMs = 3000;
static const size_t kDataChunk = 0x400000;

UsbDevice::UsbDevice(UsbManager* mng, libusb_device* dev, uint16_t vid, uint16_t pid, uint8_t bus, uint8_t addr,
                     bool watchdog)
    : mng_(mng), dev_(dev), vid_(vid), pid_(pid), bus_(bus), addr_(addr), watchdog_(watchdog)
{
    libusb_ref_device(dev_);
}

UsbDevice::~UsbDevice()
{
    Close();
    libusb_unref_device(dev_);
}

std::string UsbDevice::DesId() const
{
    std::string s = info.name;
    s += '(';
    s += (char)addr_;
    s += ')';
    return s;
}

bool UsbDevice::QueryEndpoints()
{
    libusb_device_descriptor dd;
    if (libusb_get_device_descriptor(dev_, &dd) < 0)
        return false;
    for (uint8_t c = 0; c < dd.bNumConfigurations; c++) {
        libusb_config_descriptor* cfg = nullptr;
        if (libusb_get_config_descriptor(dev_, c, &cfg) != 0)
            continue;
        for (int i = 0; i < cfg->bNumInterfaces; i++) {
            const libusb_interface& itf = cfg->interface[i];
            for (int a = 0; a < itf.num_altsetting; a++) {
                const libusb_interface_descriptor& id = itf.altsetting[a];
                for (int e = 0; e < id.bNumEndpoints; e++) {
                    const libusb_endpoint_descriptor& ep = id.endpoint[e];
                    if ((ep.bmAttributes & 3) != LIBUSB_TRANSFER_TYPE_BULK)
                        continue;
                    uint8_t adr = ep.bEndpointAddress;
                    if (adr & 0x80) {
                        if (adr == 0x81)
                            ep_ctrl_in_ = adr;
                        else
                            ep_data_in_ = adr;
                    } else {
                        if (adr == 0x01)
                            ep_ctrl_out_ = adr;
                        else
                            ep_data_out_ = adr;
                    }
                }
            }
        }
        libusb_free_config_descriptor(cfg);
    }
    return true;
}

// Synchronous check that the device answers the serial reset; also drops
// replies left over from a previous session.
bool UsbDevice::Preflight()
{
    static const uint8_t kReset[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55};
    uint8_t out[10];
    std::memcpy(out, kReset, 10);
    int tr = 0;
    if (libusb_bulk_transfer(h_, ep_ctrl_out_, out, 10, &tr, 500) < 0)
        return false;
    for (int i = 0; i < 16; i++) {
        uint8_t in[1024];
        tr = 0;
        if (libusb_bulk_transfer(h_, ep_ctrl_in_, in, sizeof(in), &tr, 500) < 0)
            return false;
        if (tr >= 10 && std::memcmp(in, kReset, 10) == 0)
            return true;
        LOG("dropped %d stale reply bytes\n", tr);
    }
    return false;
}

int UsbDevice::Open()
{
    if (open_)
        return -1;
    int r = libusb_open(dev_, &h_);
    if (r != 0) {
        LOG("libusb_open failed: %s\n", libusb_error_name(r));
        return -2;
    }
    QueryEndpoints();
    // Only select the configuration when needed: a SET_CONFIGURATION resets the
    // host data toggles but not the firmware ones, so after a session that left
    // an odd number of packets on an endpoint the device would never answer.
    int cfg = -1;
    if (libusb_get_configuration(h_, &cfg) < 0 || cfg != 1) {
        r = libusb_set_configuration(h_, 1);
        if (r < 0)
            LOG("libusb_set_configuration(1) failed: %s\n", libusb_error_name(r));
    }
    libusb_release_interface(h_, 0);
    r = libusb_claim_interface(h_, 0);
    if (r < 0) {
        LOG("libusb_claim_interface(0) failed: %s\n", libusb_error_name(r));
        libusb_close(h_);
        h_ = nullptr;
        return -3;
    }
    if (!ep_ctrl_in_ || !ep_data_in_ || !ep_ctrl_out_ || !ep_data_out_) {
        LOG("device endpoints not found\n");
        libusb_release_interface(h_, 0);
        libusb_close(h_);
        h_ = nullptr;
        return -3;
    }
    if (!Preflight()) {
        // the device is stuck (e.g. toggles desynchronised by another driver): reset the port
        LOG("device not answering, resetting it\n");
        libusb_release_interface(h_, 0);
        r = libusb_reset_device(h_);
        if (r < 0 || libusb_claim_interface(h_, 0) < 0 || !Preflight()) {
            LOG("device reset failed: %s\n", libusb_error_name(r));
            libusb_close(h_);
            h_ = nullptr;
            return -4;
        }
    }
    stop_ = false;
    comm_error = 0;
    thread_done_ = false;
    open_ = true;
    worker_ = std::thread(&UsbDevice::WorkerThread, this);
    return 0;
}

void UsbDevice::Close()
{
    hs_timer_.Stop();
    if (heart_started_ && open_ && !comm_error) {
        // tell the watchdog we are leaving
        Payload p(BigEndian());
        p.u32(6000);
        heart_stop_ = true;
        StaticCmd(0xfe, 0, &p, true);
        for (int i = 0; i < 20 && QueueSize(); i++)
            SleepMs(10);
    }
    heart_timer_.Stop();
    if (!open_)
        return;
    // let the modules' stop commands reach the device
    WaitIdle(3000);
    {
        std::lock_guard<std::mutex> lk(qm_);
        stop_ = true;
        qcv_.notify_all();
    }
    if (worker_.joinable()) {
        if (worker_.get_id() == std::this_thread::get_id())
            worker_.detach();
        else
            worker_.join();
    }
    open_ = false;
    if (h_) {
        libusb_release_interface(h_, 0);
        libusb_close(h_);
        h_ = nullptr;
    }
}

void UsbDevice::AddCmd(Command&& c, bool top)
{
    if (!open_)
        return;
    std::lock_guard<std::mutex> lk(qm_);
    if (top)
        queue_.push_front(std::move(c));
    else
        queue_.push_back(std::move(c));
    qcv_.notify_one();
}

void UsbDevice::Cmd(uint8_t mod, uint8_t ch, uint8_t cmd, uint32_t value, const Payload* p, bool top)
{
    AddCmd(MakeCommand(BigEndian(), mod, ch, cmd, value, p ? &p->bytes() : nullptr), top);
}

void UsbDevice::StaticCmd(uint8_t cmd, uint32_t value, const Payload* p, bool top)
{
    Cmd(0, 0, cmd, value, p, top);
}

void UsbDevice::DestroyAllCmd(int module)
{
    std::lock_guard<std::mutex> lk(qm_);
    if (module < 0) {
        queue_.clear();
        return;
    }
    for (auto it = queue_.begin(); it != queue_.end();) {
        if (it->module() == (uint8_t)module)
            it = queue_.erase(it);
        else
            ++it;
    }
}

bool UsbDevice::WaitIdle(unsigned ms)
{
    for (unsigned t = 0; t < ms; t += 10) {
        if (comm_error || thread_done_ || (!QueueSize() && !busy_))
            return true;
        SleepMs(10);
    }
    return false;
}

size_t UsbDevice::QueueSize()
{
    std::lock_guard<std::mutex> lk(qm_);
    return queue_.size();
}

void UsbDevice::RegisterModule(uint8_t module, ModuleSink* s)
{
    std::lock_guard<std::mutex> lk(sinks_m_);
    for (auto& e : sinks_)
        if (e.first == module && e.second == s)
            return;
    sinks_.emplace_back(module, s);
}

void UsbDevice::UnregisterModule(uint8_t module, ModuleSink* s)
{
    std::lock_guard<std::mutex> lk(sinks_m_);
    for (auto it = sinks_.begin(); it != sinks_.end();) {
        if (it->first == module && it->second == s)
            it = sinks_.erase(it);
        else
            ++it;
    }
    if (data_sink_ == s)
        data_sink_ = nullptr;
}

void UsbDevice::SetDataSink(ModuleSink* s)
{
    std::lock_guard<std::mutex> lk(sinks_m_);
    data_sink_ = s;
}

void UsbDevice::Fail(int code)
{
    if (!comm_error)
        comm_error = code;
    stop_ = true;
    if (on_comm_error)
        on_comm_error(code);
}

void UsbDevice::WorkerThread()
{
    for (;;) {
        Command c;
        {
            std::unique_lock<std::mutex> lk(qm_);
            qcv_.wait(lk, [this] { return stop_ || !queue_.empty(); });
            if (stop_)
                break;
            c = std::move(queue_.front());
            queue_.pop_front();
            busy_ = true;
        }
        if (c.pre_delay_ms)
            SleepMs(c.pre_delay_ms);
        if (stop_)
            break;
        CtrlOut(c);
        if (c.post_delay_ms)
            SleepMs(c.post_delay_ms);
        if (stop_)
            break;
        if (c.ask == Ask::DataIn) {
            DatasIn(c);
        } else {
            if (c.ask == Ask::DataOut)
                DatasOut(c);
            if (stop_)
                break;
            CtrlIn();
        }
        busy_ = false;
        if (stop_)
            break;
    }
    busy_ = false;
    thread_done_ = true;
}

void UsbDevice::CtrlOut(const Command& c)
{
    int tr = 0;
    int r = libusb_bulk_transfer(h_, ep_ctrl_out_, const_cast<uint8_t*>(c.pkt.data()), (int)c.pkt.size(), &tr,
                                 kUsbTimeoutMs);
    if (r < 0) {
        LOG("CtrlOut failed: %s\n", libusb_error_name(r));
        Fail(COMM_ERROR_CTRLOUT);
    }
}

void UsbDevice::CtrlIn()
{
    uint8_t buf[1024];
    std::memset(buf, 0, sizeof buf);
    int tr = 0;
    int r = libusb_bulk_transfer(h_, ep_ctrl_in_, buf, sizeof buf, &tr, kUsbTimeoutMs);
    if (r < 0) {
        LOG("CtrlIn failed: %s\n", libusb_error_name(r));
        Fail(COMM_ERROR_CTRLIN);
    }
    if (open_ && !stop_)
        OnCtrlReply(buf, (size_t)tr, r >= 0);
}

void UsbDevice::DatasOut(const Command& c)
{
    int tr = 0;
    int r = libusb_bulk_transfer(h_, ep_data_out_, c.data, (int)c.data_len, &tr, kUsbTimeoutMs);
    if (r < 0) {
        LOG("DatasOut failed: %s\n", libusb_error_name(r));
        Fail(COMM_ERROR_DATAOUT);
    }
}

void UsbDevice::DatasIn(const Command& c)
{
    size_t total = 0;
    bool ok = true;
    for (size_t off = 0; off < c.data_len; off += kDataChunk) {
        int want = (int)std::min(kDataChunk, c.data_len - off);
        int tr = 0;
        int r = libusb_bulk_transfer(h_, ep_data_in_, c.data + off, want, &tr, kUsbTimeoutMs);
        if (r < 0) {
            LOG("DatasIn failed: %s\n", libusb_error_name(r));
            ok = false;
            total = 0;
            break;
        }
        total += (size_t)tr;
    }
    if (!ok || total == 0)
        Fail(COMM_ERROR_DATAIN);
    if (open_ && !stop_)
        DispatchData(c.data, total, total != 0);
}

void UsbDevice::DispatchReply(const uint8_t* p, size_t n, bool ok)
{
    if (n == 0)
        return;
    ModuleSink* s = nullptr;
    {
        std::lock_guard<std::mutex> lk(sinks_m_);
        for (auto& e : sinks_)
            if (e.first == p[0]) {
                s = e.second;
                break;
            }
    }
    if (s)
        s->OnReply(p, n, ok);
}

void UsbDevice::DispatchData(const uint8_t* p, size_t n, bool ok)
{
    ModuleSink* s;
    {
        std::lock_guard<std::mutex> lk(sinks_m_);
        s = data_sink_;
    }
    if (s)
        s->OnData(p, n, ok);
}

// ---------------------------------------------------------------- handshake

void UsbDevice::SendResetSerial()
{
    static const uint8_t kReset[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55};
    state_ = HS_RESET;
    DestroyAllCmd(-1);
    Command c;
    c.pkt.assign(kReset, kReset + 10);
    AddCmd(std::move(c));
}

void UsbDevice::StartHandShake()
{
    SendResetSerial();
    hs_ticks_ = 0;
    hs_failures_ = 0;
    if (!hs_timer_.IsRunning())
        hs_timer_.Start(200, [this] { return HandshakeTimer(); });
}

bool UsbDevice::HandshakeTimer()
{
    if (state_ == HS_OK) {
        if (mng_)
            mng_->DeviceStateChanged(this, HS_OK);
        return false;
    }
    if (hs_failures_ < 2 && ++hs_ticks_ > 19) {
        LOG("handshake timeout (state %x)\n", (unsigned)state_);
        state_ = HS_FAILED;
        if (mng_)
            mng_->DeviceStateChanged(this, HS_FAILED);
        return false;
    }
    return true;
}

void UsbDevice::OnCtrlReply(const uint8_t* p, size_t n, bool ok)
{
    if (state_ == HS_OK) {
        DispatchReply(p, n, ok);
        return;
    }
    HandshakeStep(p, n);
    if (state_ == HS_FAILED && ++hs_failures_ <= 1) {
        LOG("handshake failed, retrying\n");
        SendResetSerial();
    }
}

void UsbDevice::HandshakeStep(const uint8_t* p, size_t n)
{
    auto is = [&](uint8_t cmd) { return n >= 4 && p[0] == 0 && p[1] == 0 && p[2] == cmd; };
    switch ((uint32_t)state_) {
    case HS_RESET: {
        static const uint8_t kEcho[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55};
        if (n >= 10 && std::memcmp(p, kEcho, 10) == 0) {
            hs_failures_ = 0;
            state_ = HS_CHALLENGE;
            Command c;
            for (int i = 0; i < 5; i++) {
                // The device does not care about the values; mimic the vendor library (rand()).
                challenge_[i] = (uint8_t)std::rand();
                c.pkt.push_back(challenge_[i]);
            }
            AddCmd(std::move(c));
        } else {
            state_ = HS_FAILED;
        }
        return;
    }
    case HS_CHALLENGE: {
        if (n < 5) {
            state_ = HS_FAILED;
            return;
        }
        std::memcpy(response_, p, 5);
        state_ = HS_RESPONSE;
        Command c;
        c.pkt.assign(challenge_, challenge_ + 5);
        c.pkt.insert(c.pkt.end(), response_, response_ + 5);
        AddCmd(std::move(c));
        return;
    }
    case HS_RESPONSE:
        if (n == 4 && p[0] == 0xaa && p[1] == 0x55 && p[2] == 0x55 && p[3] == 0xaa) {
            state_ = HS_IDENTIFY;
            StaticCmd(0xfd, 0);
        } else {
            state_ = HS_FAILED;
        }
        return;
    case HS_IDENTIFY:
        if (!info.ParseIdentify(p, n)) {
            state_ = HS_FAILED;
            return;
        }
        LOG("identified device:\n%s", info.Describe().c_str());
        state_ = HS_ADC_ADJUST;
        if (info.osc.ChannelNum()) {
            StaticCmd(0xfc, 0);
        } else if (info.dds.ChannelNum()) {
            state_ = HS_ZOOM;
            StaticCmd(0xf0, 0);
        }
        return;
    case HS_ADC_ADJUST:
        if (!info.ApplyAdcAdjust(p, n)) {
            state_ = HS_FAILED;
            return;
        }
        state_ = HS_INPUT_ADJUST;
        StaticCmd(0xfb, 0);
        return;
    case HS_INPUT_ADJUST:
        if (!info.ApplyInputAdjust(p, n)) {
            state_ = HS_FAILED;
            return;
        }
        state_ = HS_ZOOM_LENGTH;
        StaticCmd(0xfa, 0);
        return;
    case HS_ZOOM_LENGTH:
        if (!is(0xfa) || p[3] != 0 || n < 10) {
            state_ = HS_FAILED;
            return;
        }
        zoom_buf_.assign(RdBe16(p + 4), 0);
        zoom_stride_ = RdBe16(p + 6);
        zoom_chunks_ = RdBe16(p + 8);
        zoom_index_ = 1;
        state_ = HS_ZOOM;
        StaticCmd(0xfa, zoom_index_);
        return;
    default:
        break;
    }

    // Remaining calibration stages are driven by the reply command byte.
    auto next_after_dds = [&]() {
        if (!info.io.dac.empty())
            StaticCmd(0xf2, 0);
        else if (info.power.channel_num)
            StaticCmd(0xf4, 0);
        else
            HandshakeSuccess();
    };
    if (n < 4 || p[0] != 0 || p[1] != 0) {
        state_ = HS_FAILED;
        return;
    }
    switch (p[2]) {
    case 0xfa: {
        if (p[3] >= 1) {
            size_t off = (size_t)(p[3] - 1) * zoom_stride_;
            size_t len = n - 4;
            if (off + len > zoom_buf_.size())
                zoom_buf_.resize(off + len);
            std::memcpy(zoom_buf_.data() + off, p + 4, len);
        }
        if (zoom_index_ == zoom_chunks_)
            info.ApplyZoomCalibration(zoom_buf_.data(), zoom_buf_.size());
        zoom_index_++;
        if (zoom_index_ <= zoom_chunks_)
            StaticCmd(0xfa, zoom_index_);
        else if (info.dds.ChannelNum())
            StaticCmd(0xf0, 0);
        else
            next_after_dds();
        return;
    }
    case 0xf0:
        state_ = HS_DDS_ZOOM;
        if (info.ApplyDdsDacVol(p, n) && info.dds.ChannelNum())
            StaticCmd(0xf1, 0);
        else
            next_after_dds();
        return;
    case 0xf1:
        info.ApplyDdsDacZoom(p, n);
        state_ = HS_DDS_DONE;
        next_after_dds();
        return;
    case 0xf2:
        info.ApplyIoDac(p, n);
        state_ = HS_IO_DAC;
        if (info.power.channel_num)
            StaticCmd(0xf4, 0);
        else
            HandshakeSuccess();
        return;
    case 0xf4:
        info.ApplyPower(p, n);
        state_ = HS_POWER;
        HandshakeSuccess();
        return;
    default:
        state_ = HS_FAILED;
        return;
    }
}

void UsbDevice::HandshakeSuccess()
{
    LOG("handshake ok: %s\n", info.name.c_str());
    state_ = HS_OK;
    if (info.name.compare(0, 6, "MSO10B") == 0)
        Cmd(MODULE_OSC, 1, 1, 1);
    if (watchdog_) {
        Payload p(BigEndian());
        p.u32(6000);
        heart_started_ = true;
        heart_stop_ = false;
        StaticCmd(0xfe, 1, &p);
        heart_timer_.Start(1200, [this] { return HeartTimer(); });
    }
}

bool UsbDevice::HeartTimer()
{
    if (heart_stop_ || !open_ || comm_error)
        return false;
    StaticCmd(0xfe, 2);
    return true;
}

} // namespace vmopen
