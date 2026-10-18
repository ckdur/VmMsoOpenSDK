#include "stream.h"

#include <cmath>
#include <cstring>

#include "log.h"

namespace vmopen {

StreamWave* StreamWave::Create(UsbDevice* dev)
{
    const DevInfo& di = dev->info;
    if (!(di.modules & MOD_OSC) || di.osc.ChannelNum() == 0)
        return nullptr;
    const std::string& n = di.name;
    if (n.compare(0, 5, "MSO10") && n.compare(0, 5, "MSO20") && n.compare(0, 5, "MSO21") && n.compare(0, 5, "MSO41"))
        return nullptr;
    return new StreamWave(dev);
}

StreamWave::StreamWave(UsbDevice* dev) : dev_(dev)
{
    const OscDesc& o = dev_->info.osc;
    ch_.resize(o.ChannelNum());
    for (auto& c : ch_) {
        c.min_mv = (int)(((double)o.range_min_mv / 1000.0) * 1000.0);
        c.max_mv = (int)(((double)o.range_max_mv / 1000.0) * 1000.0);
    }
    // Number of sample rates usable in stream mode (USB bandwidth bound)
    max_index_ = (dev_->info.hw_ver > 1 ? 1 : 0) + 12;
    if (dev_->info.name.compare(0, 5, "MSO20") != 0)
        max_index_ = 12;
    FillSamples();
    rate_ = rates_.empty() ? 0 : rates_[0];
    th_ = std::thread(&StreamWave::DataThread, this);
}

StreamWave::~StreamWave()
{
    running_ = false;
    {
        std::lock_guard<std::mutex> lk(qm_);
        exit_ = true;
        qcv_.notify_all();
    }
    if (th_.joinable())
        th_.join();
    dev_->UnregisterModule(MODULE_OSC, this);
}

void StreamWave::Cmd(uint8_t ch, uint8_t cmd, uint32_t value, const Payload* p)
{
    dev_->Cmd(MODULE_OSC, ch, cmd, value, p);
}

unsigned StreamWave::CheckedNum() const
{
    unsigned n = 0;
    for (auto& c : ch_)
        n += c.checked;
    return n;
}

unsigned StreamWave::BlockAlign() const
{
    unsigned per = BytesPerSample();
    unsigned lb = dev_->info.logic.channel_num >> 3;
    unsigned m = per < lb ? per : lb;
    unsigned n = CheckedNum() * per;
    if (logic_mask_)
        n += lb;
    return (int)n <= (int)m ? m : n;
}

void StreamWave::FillSamples()
{
    unsigned align = BlockAlign();
    int k;
    if ((int)align < 2)
        k = (int)max_index_ + 1;
    else if (align == 2)
        k = (int)max_index_;
    else if (align == 4)
        k = (int)max_index_ - 1;
    else if (align == 8)
        k = (int)max_index_ - 2;
    else if (align == 16)
        k = (int)max_index_ - 3;
    else
        return;
    const auto& r = dev_->info.osc.sample_rates;
    unsigned cnt = (unsigned)k < r.size() ? (unsigned)k : (unsigned)r.size();
    rates_.assign(r.begin(), r.begin() + cnt);
    if (!rates_.empty() && rates_.back() < rate_)
        rate_ = rates_.back();
}

unsigned StreamWave::SupportSamples(unsigned* out, int n) const
{
    const auto& r = dev_->info.osc.sample_rates;
    unsigned cnt = SupportSampleNum();
    if (cnt > r.size())
        cnt = (unsigned)r.size();
    if (cnt <= (unsigned)n || n < 0)
        n = (int)cnt;
    for (int i = 0; out && i < n; i++)
        out[i] = r[i];
    return (unsigned)n;
}

void StreamWave::SetMinRangemV(unsigned c, int mv)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c < ch_.size())
        ch_[c].min_mv = mv;
}

void StreamWave::SetMaxRangemV(unsigned c, int mv)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c < ch_.size())
        ch_[c].max_mv = mv;
}

void StreamWave::SetAcDc(unsigned c, bool ac)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].ac = ac;
    if (running_) {
        Payload p(dev_->BigEndian());
        p.u8(ac ? 1 : 0);
        Cmd((uint8_t)(c + 1), 0x19, 1, &p);
    }
}

double StreamWave::Ad2VoltagePara() const
{
    const OscDesc& o = dev_->info.osc;
    double d = std::pow(2.0, (double)o.adc_bits);
    return (o.adc_max_v - o.adc_min_v) / (d - 1.0);
}

double StreamWave::TotalZoom(unsigned c) const
{
    const OscDesc& o = dev_->info.osc;
    double i = o.InzoomValue(c, ch_[c].inzoom);
    double z = o.ZoomValue(c, ch_[c].zoom);
    return i / z;
}

double StreamWave::ADCToVoltageZoom(unsigned c) const
{
    if (c >= ch_.size())
        return 0;
    double p = Ad2VoltagePara();
    return TotalZoom(c) * p;
}

double StreamWave::ADCToVoltageBias(unsigned c) const
{
    if (c >= ch_.size())
        return 0;
    double v = dev_->info.osc.adc_min_v;
    return TotalZoom(c) * v;
}

int StreamWave::Voltage2Ad(unsigned c, double v) const
{
    const Probe& pr = ch_[c].probe;
    double bias = ADCToVoltageBias(c);
    double para = Ad2VoltagePara();
    double zoom = TotalZoom(c);
    return (int)(((v - pr.off) / pr.mul - bias) / (zoom * para));
}

// --------------------------------------------------------------- commands

void StreamWave::SendInputZoom(unsigned c)
{
    Chan& s = ch_[c];
    const OscChannelDesc& d = dev_->info.osc.ch[c];
    Payload p(dev_->BigEndian());
    p.u16((uint16_t)(int)d.Jiaozhun(s.inzoom, s.zoom));
    if (!s.inzoom_sent) {
        s.inzoom_sent = true;
        Cmd((uint8_t)(c + 1), 0x06, s.inzoom == 0, &p);
    }
    Cmd((uint8_t)(c + 1), 0x06, s.inzoom, &p);
}

void StreamWave::SendZoom(unsigned c)
{
    Chan& s = ch_[c];
    const OscChannelDesc& d = dev_->info.osc.ch[c];
    int16_t dac = s.zoom < d.zoom.size() ? d.zoom[s.zoom].dac : 0;
    Payload p(dev_->BigEndian());
    p.u16((uint16_t)(int)(((double)(dac + 0x4e2) * 0.5 * 4096.0) / 3300.0));
    p.u16((uint16_t)(int)d.Jiaozhun(s.inzoom, s.zoom));
    Cmd((uint8_t)(c + 1), 0x04, s.zoom, &p);
}

void StreamWave::SendAcDc(unsigned c)
{
    Chan& s = ch_[c];
    if (!s.acdc_sent) {
        s.acdc_sent = true;
        Payload p0(dev_->BigEndian());
        p0.u8((uint8_t)(1 - (s.ac ? 1 : 0)));
        Cmd((uint8_t)(c + 1), 0x19, 1, &p0);
    }
    Payload p(dev_->BigEndian());
    p.u8(s.ac ? 1 : 0);
    Cmd((uint8_t)(c + 1), 0x19, 1, &p);
}

void StreamWave::SendStyle(uint64_t src, unsigned style)
{
    uint32_t idx = 0;
    switch (style) {
    case 0x001: idx = 3; break;
    case 0x002: idx = 4; break;
    case 0x004: idx = 5; break;
    case 0x008: idx = 6; break;
    case 0x010: idx = 7; break;
    case 0x020: idx = 8; break;
    case 0x040: idx = 9; break;
    case 0x080: idx = 10; break;
    case 0x100: idx = 0xb; break;
    default: idx = 0; break;
    }
    int a = TrigSrcToAnalog(src), d = TrigSrcToDigital(src);
    if (a >= 0)
        Cmd((uint8_t)(a + 1), 0x18, idx);
    else if (d >= 0)
        Cmd((uint8_t)d, 0x20, idx);
}

void StreamWave::SendLevel(double level, double sense, uint64_t src)
{
    int c = TrigSrcToAnalog(src);
    if (c < 0 || (unsigned)c >= ch_.size())
        return;
    int lv = Voltage2Ad((unsigned)c, level);
    int s = Voltage2Ad((unsigned)c, level + sense) - lv;
    Payload p(dev_->BigEndian());
    p.u16((uint16_t)lv).u16((uint16_t)s);
    Cmd((uint8_t)(c + 1), 0x13, (uint32_t)(int)(level * 1000.0), &p);
}

void StreamWave::SendCapture()
{
    Payload p(dev_->BigEndian());
    p.u32((uint32_t)(block_ >> 1)).u32((uint32_t)block_);
    Cmd(0xff, 0x02, (uint32_t)block_, &p);
}

void StreamWave::SendReadData()
{
    size_t bytes = (size_t)BlockAlign() * block_;
    auto b = std::make_shared<Block>();
    b->data.resize(bytes);
    b->size = bytes;
    b->index = sent_blocks_++;
    cur_ = b;
    Command c = MakeCommand(dev_->BigEndian(), MODULE_OSC, 0xff, 0x03, (uint32_t)(bytes >> 1));
    c.ask = Ask::DataIn;
    c.data = b->data.data();
    c.data_len = bytes;
    dev_->AddCmd(std::move(c));
}

void StreamWave::StopChannels()
{
    for (unsigned c = 0; c < ch_.size(); c++)
        if (ch_[c].checked)
            Cmd((uint8_t)(c + 1), 0x10, 0);
    if (logic_mask_)
        Cmd(0xff, 0x0b, 0);
}

bool StreamWave::Start()
{
    if (running_ || (CheckedNum() == 0 && logic_mask_ == 0))
        return running_;
    const OscDesc& o = dev_->info.osc;
    dev_->RegisterModule(MODULE_OSC, this);
    dev_->SetDataSink(this);
    dev_->DestroyAllCmd(MODULE_OSC);
    Cmd(0xff, 0x12, 0);
    if (o.adc_adjust_num == 1 || o.adc_adjust_num == 2)
        for (unsigned c = 0; c < ch_.size(); c++) {
            const auto& adj = o.ch[c].adc_adjust;
            Payload p(dev_->BigEndian());
            p.u32(adj.size() > 0 ? (uint8_t)adj[0] : 0);
            p.u32(o.adc_adjust_num == 2 && adj.size() > 1 ? (uint8_t)adj[1] : 0);
            Cmd((uint8_t)(c + 1), 0x17, 8, &p);
        }
    for (unsigned c = 0; c < ch_.size(); c++)
        if (ch_[c].checked)
            Cmd((uint8_t)(c + 1), 0x01, 0);
    if (logic_mask_ == 0)
        Cmd(0xff, 0x0b, 0);
    else
        Cmd(0xff, 0x0a, logic_mask_);
    Cmd(0xff, 0x3f, 1);
    Cmd(0xff, 0x15, 1);
    {
        unsigned idx = 0;
        for (unsigned i = 0; i < o.sample_rates.size(); i++)
            if (o.sample_rates[i] == rate_)
                idx = i;
        Payload p(dev_->BigEndian());
        p.u8((uint8_t)idx);
        Cmd(0xff, 0x08, 1, &p);
    }
    Cmd(0xff, 0x11, 1);
    for (unsigned c = 0; c < ch_.size(); c++) {
        Chan& s = ch_[c];
        s.acdc_sent = false;
        if (o.support_acdc)
            SendAcDc(c);
        double vmin = (double)s.min_mv / 1000.0, vmax = (double)s.max_mv / 1000.0;
        // input attenuator / gain selection (same rules as block mode)
        double a = std::fabs(vmin);
        if (std::fabs(vmin) <= std::fabs(vmax))
            a = std::fabs(vmax);
        unsigned nin = (unsigned)o.ch[c].inzoom.size();
        unsigned in = nin - 1;
        for (unsigned i = 0; i < nin; i++)
            if (a / s.probe.mul - s.probe.off <= o.InzoomValue(c, i) * (o.adc_max_v / o.ZoomValue(c, 0)) * 0.8) {
                in = i;
                break;
            }
        s.inzoom = in;
        double iz = o.InzoomValue(c, in);
        unsigned nz = (unsigned)o.ch[c].zoom.size();
        s.zoom = 0;
        for (int i = (int)nz - 1; i >= 0; i--)
            if (o.ZoomValue(c, (unsigned)i) <= std::fabs(o.adc_max_v) / ((a / s.probe.mul - s.probe.off) / iz)) {
                s.zoom = (unsigned)i;
                break;
            }
        s.inzoom_sent = false;
        SendInputZoom(c);
        SendZoom(c);
    }
    // main / aided trigger (CH1 / CH2, style none, level 0, sense 0.5V)
    SendStyle(1, 0);
    if (ch_.size() > 0)
        SendLevel(0.0, 0.5, 1);
    SendStyle(2, 0);
    if (ch_.size() > 1)
        SendLevel(0.0, 0.5, 2);
    {
        std::lock_guard<std::mutex> lk(qm_);
        queue_.clear();
    }
    state_ = ST_RUN;
    running_ = true;
    sent_blocks_ = 0;
    unsigned align = BlockAlign();
    uint64_t mem = (uint64_t)o.memory_kb << 10;
    uint64_t x = ((uint64_t)rate_ / 1000 / count_per_s_) << 10;
    uint64_t y = (mem / align) >> 1;
    if (x < 0x400)
        x = 0x400;
    if (total_ < x)
        x = total_;
    block_ = x < y ? x : y;
    nblocks_ = total_ / block_ + (total_ % block_ ? 1 : 0);
    SendCapture();
    return true;
}

int StreamWave::Capture(unsigned count_every_1s, uint64_t length_kb, uint16_t chmask, unsigned rate)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    for (unsigned c = 0; c < ch_.size(); c++)
        ch_[c].checked = (chmask >> c) & 1;
    FillSamples();
    bool found = false;
    for (unsigned r : rates_)
        if (r == rate)
            found = true;
    if (rates_.empty() || !found)
        return -1;
    for (unsigned r : dev_->info.osc.sample_rates)
        if (r == rate)
            rate_ = rate;
    uint64_t kb = (uint64_t)(rate >> 2) / 125;
    if (kb <= length_kb)
        kb = length_kb;
    count_per_s_ = count_every_1s ? count_every_1s : 1;
    total_ = kb << 10;
    Start();
    return 0;
}

// ----------------------------------------------------------------- replies

void StreamWave::OnReply(const uint8_t* p, size_t n, bool)
{
    if (n < 4 || p[0] != MODULE_OSC || p[2] != 0x02)
        return;
    std::lock_guard<std::recursive_mutex> lk(m_);
    uint8_t st = p[3];
    if (st == 2) {
        if (n > 4 && p[4] == 1)
            state_ = ST_OVERFLOW;
        SendReadData();
    } else if (st == 0x10) {
        if (running_) {
            Cmd(0xff, 0x16, 0);
            state_ = ST_WAIT;
        } else {
            StopChannels();
        }
        Notify();
    }
}

void StreamWave::OnData(const uint8_t*, size_t n, bool ok)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (!ok || n == 0 || !cur_) {
        state_ = ST_ERROR;
    } else {
        cur_->size = n;
        {
            std::lock_guard<std::mutex> q(qm_);
            queue_.push_back(cur_);
        }
        cur_.reset();
        if (sent_blocks_ == nblocks_) {
            state_ = ST_DONE;
            running_ = false;
        } else if (state_ == ST_WAIT) {
            state_ = ST_RUN;
        } else if (state_ == ST_DONE || state_ == ST_OVERFLOW) {
            running_ = false;
        }
    }
    Notify();
    if (!running_) {
        Cmd(0xff, 0x15, 0);
        StopChannels();
    } else {
        SendCapture();
    }
}

void StreamWave::Notify()
{
    std::lock_guard<std::mutex> lk(qm_);
    notified_ = true;
    qcv_.notify_one();
}

void StreamWave::DataThread()
{
    for (;;) {
        std::deque<std::shared_ptr<Block>> blocks;
        int st;
        {
            std::unique_lock<std::mutex> lk(qm_);
            qcv_.wait(lk, [this] { return exit_ || notified_; });
            if (exit_)
                return;
            notified_ = false;
            st = state_;
            if (st == ST_OVERFLOW || st == ST_ERROR) {
                queue_.clear();
                state_ = ST_RUN;
                continue;
            }
            if (st == ST_WAIT)
                continue;
            blocks.swap(queue_);
        }
        unsigned align, nch;
        std::vector<double> zoom, bias;
        std::vector<int> off;
        uint64_t need, blk;
        {
            std::lock_guard<std::recursive_mutex> lk(m_);
            align = BlockAlign();
            nch = (unsigned)ch_.size();
            int o = 0;
            for (unsigned c = 0; c < nch; c++) {
                zoom.push_back(ADCToVoltageZoom(c));
                bias.push_back(ADCToVoltageBias(c));
                off.push_back(ch_[c].checked ? o : -1);
                if (ch_[c].checked)
                    o += (int)BytesPerSample();
            }
            need = total_;
            blk = block_;
        }
        size_t left = blocks.size();
        std::vector<double> v;
        for (auto& b : blocks) {
            left--;
            uint8_t* d = b->data.data();
            if (b->index == 0 && b->size >= 6) {
                if (align == 2) {
                    d[0] = d[4];
                    d[2] = d[4];
                    d[1] = d[5];
                    d[3] = d[5];
                } else if (align == 1) {
                    d[0] = d[2];
                    d[1] = d[2];
                }
            }
            unsigned n = align ? (unsigned)(b->size / align) : 0;
            StreamBlockInfo info;
            info.failed = 0;
            info.success = (state_ == ST_DONE && left == 0) ? 1 : 0;
            info.need_total = need;
            info.total = b->index * blk + n;
            info.memory = left * b->size;
            v.resize(n);
            for (unsigned c = 0; c < nch; c++) {
                if (off[c] < 0)
                    continue;
                for (unsigned i = 0; i < n; i++) {
                    unsigned raw = d[(size_t)i * align + off[c]];
                    if (BytesPerSample() == 2)
                        raw |= (unsigned)d[(size_t)i * align + off[c] + 1] << 8;
                    v[i] = (double)raw * zoom[c] + bias[c];
                }
                if (on_data)
                    on_data(c, v.data(), n, info);
            }
            if (on_raw)
                on_raw(d, (unsigned)b->size, info);
        }
    }
}

} // namespace vmopen
