#include "dds.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include "log.h"
#include "timer.h"

namespace vmopen {

static inline uint16_t ToCode(double x) { return (uint16_t)(short)(int)x; }

void GenerateDdsBuffer(uint16_t* out, int n, unsigned style, double phase, double duty, double p, double period_ns,
                       double delay_ns, double rise_ns, double fall_ns, short dac_min, short dac_max, int vol_min,
                       int vol_max, bool invert, int ampl, int bias)
{
    int lo = vol_min, hi = vol_max;
    if (invert) {
        lo = -vol_max;
        bias = -bias;
        hi = -vol_min;
    }
    double span = (double)((int)dac_max - (int)dac_min);
    double A = (((double)ampl / p) * span) / (double)(hi - lo);
    double C = (((double)(bias - lo) / p) * span) / (double)(hi - lo) + (double)(int)dac_min;
    bool generated = true;
    switch (style) {
    case BOX_SINE:
        for (int i = 0; i < n; i++) {
            double s = std::sin(((((double)i * 360.0) / (double)n) * 3.14159265) / 180.0);
            out[i] = ToCode(s * A * 0.5 + C + 0.5);
        }
        break;
    case BOX_SQUARE: {
        int k = (int)((duty * (double)n) / 100.0);
        if (k > n)
            k = n;
        int i = 0;
        for (; i < k; i++)
            out[i] = ToCode(A * 0.5 + C);
        for (; i < n; i++)
            out[i] = ToCode(C - A * 0.5);
        break;
    }
    case BOX_RAMP: {
        int k = (int)((duty * (double)(n / 2)) / 100.0);
        double half = A * 0.5;
        int pos = 0;
        if (k >= 1) {
            double d = (double)(k - 1);
            for (int j = 0; j < k; j++)
                out[j] = ToCode(((double)j * half) / d + C);
            pos = k;
        }
        int e = n - k;
        if (pos < e) {
            double d = (double)((e - k) - 1);
            for (int j = pos - k; pos < e; j++, pos++)
                out[pos] = ToCode((half - ((double)j * A) / d) + C);
        }
        if (pos < n) {
            double d = (double)(k - 1);
            for (int t = (n - 1) - pos; t >= 0; t--, pos++)
                out[pos] = ToCode(((double)t * -half) / d + C);
        }
        break;
    }
    case BOX_PULSE: {
        if (period_ns < delay_ns + rise_ns + fall_ns) {
            generated = false;
            break;
        }
        double dn = (double)n;
        int nr = (int)((dn * rise_ns) / period_ns);
        int nf = (int)((dn * fall_ns) / period_ns);
        int nd = (int)((dn * delay_ns) / period_ns);
        int nh = ((int)((dn * duty) / 100.0) - nr) - nf;
        int pos = 0;
        uint16_t low = ToCode(C - A * 0.5), high = ToCode(C + A * 0.5);
        for (; pos < nd && pos < n; pos++)
            out[pos] = low;
        int e = nd + nr;
        if (pos < e) {
            double d = (double)(nr - 1);
            for (int j = pos - nd; pos < e && pos < n; j++, pos++)
                out[pos] = ToCode((((double)j * A) / d - A * 0.5) + C);
        }
        int e2 = (nh > 0 ? nh : 0) + e;
        int end = e2 <= n ? e2 : n;
        for (; pos < end; pos++)
            out[pos] = high;
        int e3 = nf + e2;
        if (e3 > n)
            e3 = n;
        if (pos < e3) {
            double d = (double)(nf - 1);
            for (int j = pos - e2; pos < e3; j++, pos++)
                out[pos] = ToCode((A * 0.5 - ((double)j * A) / d) + C);
        }
        for (; pos < n; pos++)
            out[pos] = low;
        break;
    }
    case BOX_NOISE: {
        std::srand((unsigned)std::time(nullptr));
        int a = (int)A;
        for (int i = 0; i < n; i++) {
            int r = a ? std::rand() % a : 0;
            out[i] = ToCode((double)(r - a / 2) + C);
        }
        break;
    }
    case BOX_DC:
        for (int i = 0; i < n; i++)
            out[i] = ToCode(C);
        break;
    default:
        generated = false;
        break;
    }
    (void)generated;
    if (invert)
        for (int i = 0; i < n; i++)
            out[i] = ToCode((C + C) - (double)out[i]);
    if (phase > 0.0 && phase < 360.0 && n > 0) {
        int shift = (int)(((double)n * phase) / 360.0);
        if (shift > 0 && shift < n) {
            std::vector<uint16_t> tmp(out, out + shift);
            std::memmove(out, out + shift, (size_t)(n - shift) * 2);
            std::memcpy(out + (n - shift), tmp.data(), (size_t)shift * 2);
        }
    }
}

void GenerateDdsArbBuffer(uint16_t* out, int n, short dac_min, short dac_max, int vol_min, int vol_max, bool invert,
                          int ampl, int bias, double p, const uint16_t* src)
{
    int lo = vol_min, hi = vol_max;
    if (invert) {
        lo = -vol_max;
        bias = -bias;
        hi = -vol_min;
        for (int i = 0; i < n; i++)
            out[i] = (uint16_t)(dac_max - (short)src[i]);
    } else {
        for (int i = 0; i < n; i++)
            out[i] = src[i];
    }
    double A = (double)ampl;
    double span = (double)((int)dac_max - (int)dac_min);
    double dlo = (double)(int)dac_min;
    double dmax = (double)(int)dac_max;
    for (int i = 0; i < n; i++) {
        double x = (double)out[i];
        double v = (((double)(bias - lo) / p) * span) / (double)(hi - lo) + dlo +
                   ((x - dmax * 0.5) / (dmax * 0.5)) * (((A / p) * span) / (double)(hi - lo)) * 0.5 + 0.5;
        out[i] = ToCode(v);
    }
}

// ------------------------------------------------------------- DdsDevice

DdsDevice* DdsDevice::Create(UsbDevice* dev)
{
    const DevInfo& di = dev->info;
    if (!(di.modules & MOD_DDS) || di.dds.ChannelNum() == 0)
        return nullptr;
    return new DdsDevice(dev);
}

DdsDevice::DdsDevice(UsbDevice* dev) : dev_(dev)
{
    const DdsDesc& d = dev_->info.dds;
    ch_.resize(d.ChannelNum());
    for (auto& c : ch_) {
        c.style = d.boxing.empty() ? (unsigned)BOX_SINE : (unsigned)d.boxing[0].style;
        c.buf.assign((size_t)d.depth_kb << 10, 0);
        c.arb.assign((size_t)d.depth_kb << 10, 0);
    }
}

DdsDevice::~DdsDevice()
{
    for (unsigned c = 0; c < ch_.size(); c++)
        if (ch_[c].started)
            ChannelStop(c);
    dev_->UnregisterModule(MODULE_DDS, this);
}

void DdsDevice::Cmd(uint8_t ch, uint8_t cmd, uint32_t value, const Payload* p, bool top)
{
    dev_->Cmd(MODULE_DDS, ch, cmd, value, p, top);
}

unsigned DdsDevice::PowerNum() const
{
    unsigned n = 0;
    for (auto& c : ch_)
        n += c.started;
    return n;
}

int DdsDevice::CalFengFengZhi(double freq, int mv) const
{
    const DdsDesc& d = dev_->info.dds;
    int n = d.zoom_num;
    if (n == 0 || d.zoom.empty())
        return mv;
    int i = n - 1;
    for (; i >= 0; i--)
        if ((double)d.zoom[i].freq <= freq)
            break;
    if (i == -1)
        return mv;
    if (i < n - 1) {
        const DdsFreqZoom& a = d.zoom[i];
        const DdsFreqZoom& b = d.zoom[i + 1];
        return (int)((((freq - (double)a.freq) * (b.zoom - a.zoom)) / (double)(b.freq - a.freq) + a.zoom) * (double)mv);
    }
    return (int)((double)mv * d.zoom[n - 1].zoom);
}

void DdsDevice::UpdateBufferCmd(unsigned c)
{
    const DdsDesc& d = dev_->info.dds;
    DdsChannel& s = ch_[c];
    short dmax = (short)(int)(std::pow(2.0, (double)d.dac_bits) - 1.0);
    int vmin = d.vol_min_mv[c], vmax = d.vol_max_mv[c];
    int n = (int)s.buf.size();
    if (s.style != BOX_ARB)
        GenerateDdsBuffer(s.buf.data(), n, s.style, s.phase, s.duty, 1.0, 1000000000.0 / s.freq, s.delay_ns,
                          s.rise_ns, s.fall_ns, 0, dmax, vmin, vmax, true, s.ampl_corr_mv, s.bias_mv);
    else
        GenerateDdsArbBuffer(s.buf.data(), n, 0, dmax, vmin, vmax, true, s.ampl_corr_mv, s.bias_mv, 1.0,
                             s.arb.data());
    auto keep = std::make_shared<std::vector<uint8_t>>((size_t)n * 2);
    for (int i = 0; i < n; i++) {
        (*keep)[i * 2] = (uint8_t)s.buf[i];
        (*keep)[i * 2 + 1] = (uint8_t)(s.buf[i] >> 8);
    }
    Command cmd = MakeCommand(dev_->BigEndian(), MODULE_DDS, (uint8_t)(c + 1), 0x49, (uint32_t)(n * 2));
    cmd.ask = Ask::DataOut;
    cmd.keep = keep;
    cmd.data = keep->data();
    cmd.data_len = keep->size();
    dev_->AddCmd(std::move(cmd));
}

void DdsDevice::SendFreqCmd(unsigned c, double f)
{
    const DdsDesc& d = dev_->info.dds;
    double k = std::pow(2.0, (double)d.freq_bits);
    double v = ((k - 1.0) / (double)d.clock) * f;
    uint64_t w = v >= 9.223372036854776e18 ? ((uint64_t)(int64_t)(v - 9.223372036854776e18) ^ 0x8000000000000000ull)
                                            : (uint64_t)v;
    Payload p(dev_->BigEndian());
    p.u64(w);
    Cmd((uint8_t)(c + 1), 0x42, 8, &p);
}

static uint64_t ToU64(double v)
{
    if (v >= 9.223372036854776e18)
        return (uint64_t)(int64_t)(v - 9.223372036854776e18) ^ 0x8000000000000000ull;
    return (uint64_t)v;
}

void DdsDevice::SendEnable(unsigned c, bool en) { Cmd((uint8_t)(c + 1), 0x58, en ? 1 : 0); }

void DdsDevice::UpdateSweepInfo(unsigned c)
{
    DdsChannel& s = ch_[c];
    if (s.sweep_start == s.sweep_stop)
        return;
    const DdsDesc& d = dev_->info.dds;
    double k = std::pow(2.0, (double)d.freq_bits);
    double clk = (double)d.clock;
    k = (k - 1.0) / clk;
    uint64_t a = ToU64(s.sweep_start * k);
    uint64_t b = ToU64(k * s.sweep_stop);
    uint64_t mx = a > b ? a : b, mn = a < b ? a : b;
    uint64_t step = ToU64(((double)(mx - mn) * (1000000000.0 / clk)) / (double)s.sweep_time_ns);
    if (!step)
        step = 1;
    uint64_t sa = (a / step) * step, sb = (b / step) * step;
    uint8_t chb = (uint8_t)(c + 1);
    SendEnable(c, false);
    Cmd(chb, 0x56, 0);
    Payload p(dev_->BigEndian());
    p.u64(sa).u64(sb);
    Cmd(chb, 0x43, 0x10, &p);
    Payload q(dev_->BigEndian());
    q.u64(step);
    Cmd(chb, 0x42, 8, &q);
    SendEnable(c, true);
}

void DdsDevice::UpdateBurstInfo(unsigned c)
{
    DdsChannel& s = ch_[c];
    const DdsDesc& d = dev_->info.dds;
    double clk_ns = 1000000000.0 / (double)d.clock;
    double loops = s.loops_infinity ? 0.0 : (double)s.loops;
    uint64_t cycles = ToU64((loops * (1000000000.0 / s.freq)) / clk_ns);
    uint64_t delay = ToU64((double)s.burst_delay_ns / clk_ns);
    uint64_t period = ToU64((double)s.burst_period_ns / clk_ns);
    uint8_t chb = (uint8_t)(c + 1);
    SendEnable(c, false);
    Cmd(chb, 0x56, (uint32_t)s.burst_style);
    SendFreqCmd(c, s.freq);
    Payload p(dev_->BigEndian());
    p.u64(delay).u64(cycles);
    Cmd(chb, 0x43, 0x10, &p);
    Payload q(dev_->BigEndian());
    q.u64(period);
    Cmd(chb, 0x57, 8, &q);
    SendEnable(c, true);
}

void DdsDevice::Start()
{
    if (started_)
        return;
    dev_->RegisterModule(MODULE_DDS, this);
    dev_->DestroyAllCmd(MODULE_DDS);
    Cmd(0xff, 0x12, 0);
    for (unsigned c = 0; c < ch_.size(); c++) {
        DdsChannel& s = ch_[c];
        UpdateBufferCmd(c);
        if (s.out_mode == 0)
            SendFreqCmd(c, s.freq);
        else if (s.out_mode == 1)
            UpdateSweepInfo(c);
        else
            UpdateBurstInfo(c);
        uint8_t chb = (uint8_t)(c + 1);
        Cmd(chb, 0x50, s.out_mode & 0xff);
        Cmd(chb, 0x51, s.trig_src & 0xff);
        Cmd(chb, 0x52, s.trig_enge & 0xff);
        Cmd(chb, 0x54, s.gate_enge & 0xff);
        if (s.started)
            Cmd(chb, 0x01, 0);
    }
    started_ = true;
}

void DdsDevice::ChannelStart(unsigned c)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    unsigned pn = PowerNum();
    ch_[c].started = true;
    if (pn == 0 || !started_)
        Start();
    else
        Cmd((uint8_t)(c + 1), 0x01, 0);
}

void DdsDevice::ChannelStop(unsigned c)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].started = false;
    Cmd((uint8_t)(c + 1), 0x10, 0, nullptr, true);
    if (PowerNum() == 0)
        started_ = false;
}

void DdsDevice::SetOutMode(unsigned c, unsigned mode)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].out_mode = mode;
    if (!started_)
        return;
    if (mode == 0)
        SendFreqCmd(c, ch_[c].freq);
    else if (mode == 1)
        UpdateSweepInfo(c);
    else if (mode == 2)
        UpdateBurstInfo(c);
    Cmd((uint8_t)(c + 1), 0x50, mode & 0xff);
}

void DdsDevice::SetBoxing(unsigned c, unsigned style)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].style = style;
    if (started_)
        UpdateBufferCmd(c);
}

void DdsDevice::UpdateArbBuffer(unsigned c, const uint16_t* buf, unsigned len)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size() || !buf)
        return;
    if (len > ch_[c].arb.size())
        len = (unsigned)ch_[c].arb.size();
    std::memcpy(ch_[c].arb.data(), buf, (size_t)len * 2);
    if (started_)
        UpdateBufferCmd(c);
}

void DdsDevice::SetFreq(unsigned c, double f)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    DdsChannel& s = ch_[c];
    s.freq = f;
    s.ampl_corr_mv = CalFengFengZhi(f, s.ampl_mv);
    if (!started_)
        return;
    if (s.ampl_corr_mv != s.ampl_sent_mv) {
        UpdateBufferCmd(c);
        s.ampl_sent_mv = s.ampl_corr_mv;
    }
    if (s.out_mode != 2)
        SendFreqCmd(c, f);
    else
        UpdateBurstInfo(c);
}

void DdsDevice::SetDutyCycle(unsigned c, double d)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].duty = d;
    if (started_)
        UpdateBufferCmd(c);
}

void DdsDevice::SetPhase(unsigned c, double ph)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size() || ph < 0.0 || ph > 360.0)
        return;
    ch_[c].phase = ph;
    if (started_)
        UpdateBufferCmd(c);
}

void DdsDevice::SetAmplitudeMv(unsigned c, int mv)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    DdsChannel& s = ch_[c];
    s.ampl_mv = mv;
    s.ampl_corr_mv = CalFengFengZhi(s.freq, mv);
    s.ampl_sent_mv = s.ampl_corr_mv;
    if (started_)
        UpdateBufferCmd(c);
}

void DdsDevice::SetBiasMv(unsigned c, int mv)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].bias_mv = mv;
    if (started_)
        UpdateBufferCmd(c);
}

int DdsDevice::CurBoxingAmplitudeMv(unsigned style) const
{
    const DdsBoxing* b = dev_->info.dds.Boxing(style);
    return b ? b->vol_max_mv - b->vol_min_mv : 0;
}

int DdsDevice::CurBoxingBiasMvMin(unsigned style) const
{
    const DdsBoxing* b = dev_->info.dds.Boxing(style);
    return b ? b->bias_min_mv : 0;
}

int DdsDevice::CurBoxingBiasMvMax(unsigned style) const
{
    const DdsBoxing* b = dev_->info.dds.Boxing(style);
    return b ? b->bias_max_mv : 0;
}

void DdsDevice::SetSweepStartFreq(unsigned c, double f)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].sweep_start = f;
    if (started_ && f != ch_[c].sweep_stop)
        UpdateSweepInfo(c);
}

void DdsDevice::SetSweepStopFreq(unsigned c, double f)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].sweep_stop = f;
    if (started_ && f != ch_[c].sweep_start)
        UpdateSweepInfo(c);
}

void DdsDevice::SetSweepTime(unsigned c, uint64_t ns)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].sweep_time_ns = ns;
    if (started_)
        UpdateSweepInfo(c);
}

void DdsDevice::SetBurstStyle(unsigned c, int s)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].burst_style = s;
    if (started_)
        UpdateBurstInfo(c);
}

void DdsDevice::SetLoopsNum(unsigned c, uint64_t n)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].loops = n;
    if (started_)
        UpdateBurstInfo(c);
}

void DdsDevice::SetLoopsNumInfinity(unsigned c, bool en)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].loops_infinity = en;
    if (started_)
        UpdateBurstInfo(c);
}

void DdsDevice::SetBurstPeriodNs(unsigned c, uint64_t ns)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].burst_period_ns = ns;
    if (started_)
        UpdateBurstInfo(c);
}

void DdsDevice::SetBurstDelayNs(unsigned c, uint64_t ns)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].burst_delay_ns = ns;
    if (started_)
        UpdateBurstInfo(c);
}

void DdsDevice::SetBurstLargeClkCount(unsigned c, unsigned n)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].burst_large_clk = n;
    if (started_)
        Cmd((uint8_t)(c + 1), 0x62, n);
}

void DdsDevice::SetTriggerSource(unsigned c, unsigned src)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].trig_src = src;
    if (started_)
        Cmd((uint8_t)(c + 1), 0x51, src & 0xff);
}

void DdsDevice::SetTriggerSourceIo(unsigned c, unsigned io)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].trig_io = io;
    if (started_)
        Cmd((uint8_t)(c + 1), 0x55, io);
}

void DdsDevice::SetTriggerSourceEnge(unsigned c, unsigned e)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].trig_enge = e;
    if (started_)
        Cmd((uint8_t)(c + 1), 0x52, e & 0xff);
}

void DdsDevice::SetOutputGateEnge(unsigned c, unsigned e)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].gate_enge = e;
    if (started_)
        Cmd((uint8_t)(c + 1), 0x54, e & 0xff);
}

void DdsDevice::ManualTrigger(unsigned c)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c < ch_.size() && started_)
        Cmd((uint8_t)(c + 1), 0x53, 0);
}

void DdsDevice::MultSyn()
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (started_)
        Cmd(0xff, 0x61, 0);
}

void DdsDevice::OnReply(const uint8_t* p, size_t n, bool)
{
    // The vendor library gives the output stage time to settle after a channel start.
    if (n >= 4 && p[0] == MODULE_DDS && p[2] == 0x01 && p[3] == 1)
        SleepMs(300);
}

} // namespace vmopen
