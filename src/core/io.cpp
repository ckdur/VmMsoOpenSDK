#include "io.h"

#include <cmath>

#include "log.h"

namespace vmopen {

static int MaskCount(uint32_t m)
{
    int n = 0;
    for (int i = 0; i < 8; i++)
        n += (m >> i) & 1;
    return n;
}

IoDevice* IoDevice::Create(UsbDevice* dev)
{
    const DevInfo& di = dev->info;
    if (!(di.modules & MOD_IO))
        return nullptr;
    return new IoDevice(dev);
}

IoDevice::IoDevice(UsbDevice* dev) : dev_(dev)
{
    io_.resize(dev_->info.io.io_num);
    dac_.resize(dev_->info.io.dac.size());
}

IoDevice::~IoDevice()
{
    read_timer_.Stop();
    Stop();
    dev_->UnregisterModule(MODULE_IO, this);
}

void IoDevice::Cmd(uint8_t ch, uint8_t cmd, uint32_t value, const Payload* p)
{
    dev_->Cmd(MODULE_IO, ch, cmd, value, p);
}

unsigned IoDevice::PowerNum() const
{
    unsigned n = 0;
    for (auto& l : io_)
        n += l.started;
    for (auto& d : dac_)
        n += d.power;
    return n;
}

void IoDevice::SendSetIoCtrlCmd(bool en) { Cmd(0xff, en ? 0x01 : 0x10, 0); }

void IoDevice::SendSetIoInOutCmd()
{
    uint32_t v = 0;
    for (unsigned i = 0; i < io_.size(); i++)
        if (io_[i].mask & 0x12)
            v |= 1u << i;
    Cmd(0xff, 0x30, v);
}

void IoDevice::SendSetIoPulseData(unsigned io, double hz, double duty)
{
    io_[io].pulse_hz = hz;
    io_[io].pulse_duty = duty;
    Payload p(dev_->BigEndian());
    p.u32((uint32_t)(long)(100000000.0 / hz - 1.0));
    p.u32((uint32_t)(long)(((duty * 100000000.0) / 100.0) / hz));
    Cmd((uint8_t)io, 0x34, 8, &p);
}

void IoDevice::SendSetIoStateCmd(unsigned io, uint32_t st)
{
    Cmd((uint8_t)io, 0x31, st & 0xff);
    if ((st & 0xff) == 3)
        SendSetIoPulseData(io, io_[io].pulse_hz, io_[io].pulse_duty);
}

bool IoDevice::Start()
{
    if (started_)
        return true;
    dev_->RegisterModule(MODULE_IO, this);
    dev_->DestroyAllCmd(MODULE_IO);
    SendSetIoCtrlCmd(true);
    for (unsigned i = 0; i < io_.size(); i++)
        SendSetIoStateCmd(i, io_[i].out_state);
    SendSetIoInOutCmd();
    started_ = true;
    read_timer_.Start(read_ms_, [this] { return ReadTimer(); });
    return true;
}

void IoDevice::Stop()
{
    started_ = false;
    SendSetIoCtrlCmd(false);
}

void IoDevice::StopIo(unsigned io)
{
    if (io >= io_.size())
        return;
    if (MaskCount(io_[io].mask) < 2)
        io_[io].started = false;
    if (PowerNum() != 0)
        return;
    Stop();
}

void IoDevice::SetIOPower(unsigned io, bool en)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (io >= io_.size())
        return;
    if (en) {
        if (MaskCount(io_[io].mask) < 1)
            return;
        io_[io].started = true;
        if (!started_)
            Start();
    } else {
        StopIo(io);
    }
}

bool IoDevice::SetIOInOutMask(unsigned io, uint32_t mask)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (io >= io_.size())
        return false;
    uint32_t cur = io_[io].mask;
    if (MaskCount(cur) < 2 || ((cur & 0x2d) != 0) == ((mask & 0x2d) != 0)) {
        uint32_t u = cur | mask;
        if ((u & 0x12) == 0)
            mask = u;
        io_[io].mask = mask;
        if (!started_)
            return true;
        SendSetIoInOutCmd();
        return true;
    }
    return false;
}

void IoDevice::SetIOOutState(unsigned io, uint32_t st)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (io >= io_.size())
        return;
    io_[io].out_state = st;
    if (started_)
        SendSetIoStateCmd(io, st);
}

void IoDevice::SetIoPulseData(unsigned io, double hz, double duty)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (io >= io_.size())
        return;
    io_[io].pulse_hz = hz;
    io_[io].pulse_duty = duty;
    if (started_)
        SendSetIoPulseData(io, hz, duty);
}

void IoDevice::SetIoPulseSyn()
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    Cmd(0xff, 0x35, 0);
}

void IoDevice::SendReadIOInStateCmd()
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    Cmd(0xff, 0x32, 0);
}

void IoDevice::SetDacPower(unsigned d, bool en)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (d >= dac_.size())
        return;
    if (!en) {
        dac_[d].power = false;
        if (PowerNum() == 0)
            Stop();
        return;
    }
    dac_[d].power = true;
    if (!started_)
        Start();
}

void IoDevice::SetDacmV(unsigned d, int mv)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (d >= dac_.size() || !started_)
        return;
    const IoDac& dd = dev_->info.io.dac[d];
    dac_[d].mv = mv;
    long top = (long)(int)(std::pow(2.0, (double)dd.bits) - 1.0);
    long v = ((long)(mv - dd.vol_min_mv) * top) / (long)(dd.vol_max_mv - dd.vol_min_mv);
    Cmd((uint8_t)(d + 1), 0x36, (uint32_t)v);
}

bool IoDevice::ReadTimer()
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (!started_)
        return false;
    for (auto& l : io_)
        if (l.started && (l.mask & IO_INPUT)) {
            Cmd(0xff, 0x32, 0);
            break;
        }
    return true;
}

void IoDevice::OnReply(const uint8_t* p, size_t n, bool)
{
    if (n < 4 || p[2] != 0x32)
        return;
    uint32_t st = 0xffffffff;
    if (p[0] == MODULE_IO && p[3] == 1 && n >= 6)
        st = RdBe16(p + 4);
    if (on_state)
        on_state(st);
}

} // namespace vmopen
