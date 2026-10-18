#include "osc.h"

#include <cmath>
#include <cstring>

#include "log.h"
#include "timer.h"

namespace vmopen {

OscCard* OscCard::Create(UsbDevice* dev)
{
    const DevInfo& di = dev->info;
    if (!(di.modules & MOD_OSC) || di.osc.ChannelNum() == 0)
        return nullptr;
    const std::string& n = di.name;
    auto starts = [&](const char* s) { return n.compare(0, std::strlen(s), s) == 0; };
    if (starts("MSO10"))
        return new OscCard(dev, OSC_MSO10);
    if (starts("MSO20"))
        return new OscCard(dev, di.hw_ver < 4 ? OSC_MSO20 : OSC_MSO20V4);
    if (starts("MSO21AL PLUS") || starts("MSO21BL PLUS"))
        return new OscCard(dev, OSC_MSO21PLUS);
    if (starts("MSO21"))
        return new OscCard(dev, di.hw_ver == 1 ? OSC_MSO21 : di.hw_ver == 2 ? OSC_MSO21V2 : OSC_MSO21V2_2);
    if (starts("MSO41"))
        return new OscCard(dev, OSC_MSO41);
    LOG("no oscilloscope implementation for %s\n", n.c_str());
    return nullptr;
}

OscCard::OscCard(UsbDevice* dev, OscModel model) : dev_(dev), model_(model)
{
    const OscDesc& o = dev_->info.osc;
    unsigned nch = o.ChannelNum();
    ch_.resize(nch);
    range_min_mv_.assign(nch, (double)o.range_min_mv);
    range_max_mv_.assign(nch, (double)o.range_max_mv);
    for (unsigned c = 0; c < nch; c++)
        ch_[c].inzoom = (unsigned)o.ch[c].inzoom.size() - 1;
    sample_idx_ = o.sample_rates.empty() ? 0 : (unsigned)o.sample_rates.size() - 1;
    hard_trigger_ = o.trigger_styles == 0 ? 2 : 1;
}

OscCard::~OscCard()
{
    Stop();
    dev_->UnregisterModule(MODULE_OSC, this);
}

void OscCard::Cmd(uint8_t ch, uint8_t cmd, uint32_t value, const Payload* p)
{
    dev_->Cmd(MODULE_OSC, ch, cmd, value, p);
}

// ------------------------------------------------------------- conversions

double OscCard::Ad2VoltagePara() const
{
    const OscDesc& o = dev_->info.osc;
    double d = std::pow(2.0, (double)o.adc_bits);
    return (o.adc_max_v - o.adc_min_v) / (d - 1.0);
}

double OscCard::TotalZoom(unsigned c) const
{
    const OscDesc& o = dev_->info.osc;
    double z = o.ZoomValue(c, ch_[c].zoom);
    double i = o.InzoomValue(c, ch_[c].inzoom);
    return i / z;
}

double OscCard::BiasContrl(unsigned c) const
{
    double v = dev_->info.osc.adc_min_v;
    return TotalZoom(c) * v;
}

double OscCard::VoltageResolution(unsigned c) const
{
    if (c >= ch_.size())
        return 0;
    return TotalZoom(c) * Ad2VoltagePara();
}

double OscCard::ADCToVoltageZoom(unsigned c) const
{
    if (c >= ch_.size())
        return 0;
    double p = Ad2VoltagePara();
    return TotalZoom(c) * p;
}

double OscCard::ADCToVoltageBias(unsigned c) const
{
    if (c >= ch_.size())
        return 0;
    return BiasContrl(c);
}

int OscCard::Voltage2Ad(unsigned c, double v) const
{
    const Probe& pr = ch_[c].probe;
    double x = ch_[c].invert ? -v : v;
    double bias = BiasContrl(c);
    double para = Ad2VoltagePara();
    double zoom = TotalZoom(c);
    return (int)(((x - pr.off) / pr.mul - bias) / (zoom * para));
}

unsigned OscCard::InputZoomFor(unsigned c, double min_v, double max_v) const
{
    const OscDesc& o = dev_->info.osc;
    const Probe& pr = ch_[c].probe;
    double a = std::fabs(min_v);
    if (std::fabs(min_v) <= std::fabs(max_v))
        a = std::fabs(max_v);
    unsigned n = (unsigned)o.ch[c].inzoom.size();
    for (unsigned i = 0; i < n; i++) {
        double top = o.adc_max_v;
        double z0 = o.ZoomValue(c, 0);
        double iz = o.InzoomValue(c, i);
        if (a / pr.mul - pr.off <= iz * (top / z0) * 0.8)
            return i;
    }
    return n - 1;
}

unsigned OscCard::ZoomFor(unsigned c, unsigned in, double min_v, double max_v) const
{
    const OscDesc& o = dev_->info.osc;
    const Probe& pr = ch_[c].probe;
    double a = std::fabs(min_v);
    if (std::fabs(min_v) <= std::fabs(max_v))
        a = std::fabs(max_v);
    double iz = o.InzoomValue(c, in);
    double top = o.adc_max_v, bot = o.adc_min_v;
    unsigned n = (unsigned)o.ch[c].zoom.size();
    for (int i = (int)n - 1; i >= 0; i--) {
        double z = o.ZoomValue(c, (unsigned)i);
        if (z <= ((top - bot) * 0.5) / ((a / pr.mul - pr.off) / iz))
            return (unsigned)i;
    }
    return 0;
}

unsigned OscCard::Sample2Index(unsigned rate) const
{
    const auto& r = dev_->info.osc.sample_rates;
    for (unsigned i = 0; i < r.size(); i++)
        if (r[i] == rate)
            return i;
    return 0;
}

unsigned OscCard::AnalogChannelNum(uint16_t mask) const
{
    return (mask & 1) + ((mask >> 1) & 1) + ((mask >> 2) & 1) + ((mask >> 3) & 1);
}

bool OscCard::ChannelCaptured(uint16_t mask, unsigned c) const
{
    return c < 4 && ((mask >> c) & 1);
}

// word offset of a channel inside a sample block (logic data follows the analog channels)
int OscCard::CalDataIndex(uint16_t mask, unsigned c) const
{
    if (ChannelCaptured(mask, c)) {
        int idx = -1;
        for (unsigned i = 0; i <= c; i++)
            idx += (mask >> i) & 1;
        return idx;
    }
    if (mask & 0x100)
        return (int)AnalogChannelNum(mask);
    return -1;
}

bool OscCard::Wide() const
{
    return model_ == OSC_MSO21 || model_ == OSC_MSO21V2 || model_ == OSC_MSO21V2_2 || model_ == OSC_MSO21PLUS ||
           model_ == OSC_MSO41;
}

unsigned OscCard::WordBytes() const
{
    if (!Wide())
        return 1;
    // the MSO21 Plus sends logic only captures as bytes
    return model_ == OSC_MSO21PLUS && chmask_ == 0x100 ? 1 : 2;
}

unsigned OscCard::BlockAlign() const
{
    unsigned a = AnalogChannelNum(chmask_);
    const unsigned an = a;
    const bool logic = (chmask_ & 0x100) != 0;
    const unsigned lc = dev_->info.logic.channel_num;
    switch (model_) {
    case OSC_MSO10:
        if (logic)
            a += lc >> 3;
        if (a == 3)
            a = 4;
        break;
    case OSC_MSO21:
    case OSC_MSO21V2:
    case OSC_MSO21V2_2:
        if (logic && lc)
            a += 1;
        break;
    case OSC_MSO21PLUS:
        if (chmask_ == 0x100)
            a = 1;
        else if (logic && lc)
            a += 1;
        break;
    case OSC_MSO41:
        if (logic)
            a += lc >> 4;
        if (a == 3)
            a = 4;
        if (an == 0)
            a *= 2;
        break;
    default:
        if (logic)
            a += lc >> 3;
        break;
    }
    return a;
}

unsigned OscCard::SampleAt(unsigned off, unsigned i) const
{
    size_t w = off + (size_t)i * BlockAlign();
    if (WordBytes() == 2)
        return (unsigned)buf_[2 * w] | ((unsigned)buf_[2 * w + 1] << 8);
    return buf_[w];
}

// --------------------------------------------------------------- commands

void OscCard::SendResetCmd() { Cmd(0xff, 0x12, 0); }
void OscCard::SendStartChannelCmd(uint8_t ch) { Cmd(ch, 0x01, 0); }
void OscCard::SendStopChannelCmd(uint8_t ch) { Cmd(ch, 0x10, 0); }
void OscCard::SendLogicChannelStartCmd(uint32_t mask) { Cmd(0xff, 0x0a, mask); }
void OscCard::SendLogicChannelStopCmd() { Cmd(0xff, 0x0b, 0); }
void OscCard::SendStreamModeCmd(int mode) { Cmd(0xff, 0x15, (uint32_t)mode); }
void OscCard::SendAbortCaptureCmd() { Cmd(0xff, 0x09, 1); }

void OscCard::SendSampleCtrlCmd(unsigned idx)
{
    Payload p(dev_->BigEndian());
    p.u8((uint8_t)idx);
    Cmd(0xff, 0x08, 1, &p);
}

void OscCard::SendChufaModeCmd(unsigned mode)
{
    if (mode == 2)
        mode = 1;
    Cmd(0xff, 0x11, mode);
}

static uint32_t StyleToDevIndex(unsigned style)
{
    switch (style) {
    case 0x001: return 3;
    case 0x002: return 4;
    case 0x004: return 5;
    case 0x008: return 6;
    case 0x010: return 7;
    case 0x020: return 8;
    case 0x040: return 9;
    case 0x080: return 10;
    case 0x100: return 0xb;
    default: return 0;
    }
}

void OscCard::SendChufaStyleCmd(uint64_t src, unsigned style)
{
    uint32_t idx = StyleToDevIndex(style);
    int a = TrigSrcToAnalog(src), d = TrigSrcToDigital(src), o = TrigSrcToOut(src);
    if (a >= 0)
        Cmd((uint8_t)(a + 1), 0x18, idx);
    else if (d >= 0)
        Cmd((uint8_t)d, 0x20, idx);
    else if (o >= 0)
        Cmd((uint8_t)o, 0x21, idx);
}

void OscCard::SendChufaSourceCmd(uint64_t old_src, uint64_t new_src, unsigned style)
{
    SendChufaStyleCmd(old_src, 0);
    SendChufaStyleCmd(new_src, style);
}

void OscCard::SendTriggerLevelCmd(uint8_t cmd, double level, double sense, uint64_t src)
{
    int c = TrigSrcToAnalog(src);
    if (c < 0 || (unsigned)c >= ch_.size())
        return;
    int lv = Voltage2Ad((unsigned)c, level);
    int s = Voltage2Ad((unsigned)c, level + sense);
    int d = s - lv;
    if (d < 0)
        d = -d;
    Payload p(dev_->BigEndian());
    p.u16((uint16_t)lv).u16((uint16_t)d);
    double v = (cmd == 0x13 ? level : sense) * 1000.0;
    Cmd((uint8_t)(c + 1), cmd, (uint32_t)(int)v, &p);
}

void OscCard::SendTriggerPulseNsCmd(int down, int up)
{
    Payload p(dev_->BigEndian());
    p.u32((uint32_t)down).u32((uint32_t)up);
    Cmd(0xff, 0x27, 8, &p);
}

void OscCard::SendAdcAdjustCmd(uint8_t ch, uint32_t a, uint32_t b)
{
    Payload p(dev_->BigEndian());
    p.u32(a).u32(b);
    Cmd(ch, 0x17, 8, &p);
}

void OscCard::SendCaptureCmd0(uint32_t len, uint32_t front, uint32_t time, uint32_t p4, uint32_t p5, unsigned delay)
{
    Payload p(dev_->BigEndian());
    if (front) {
        p.u32(front);
        if (time) {
            p.u32(time);
            if (p4) {
                p.u32(p4);
                if (p5)
                    p.u32(p5);
            }
        }
    }
    Command c = MakeCommand(dev_->BigEndian(), MODULE_OSC, 0xff, 0x02, len, &p.bytes());
    c.pre_delay_ms = delay;
    dev_->AddCmd(std::move(c));
}

void OscCard::SendReadDataCmd()
{
    size_t bytes;
    {
        std::lock_guard<std::recursive_mutex> lk(m_);
        bytes = (size_t)BlockBytes() * length_;
        if (buf_.size() < bytes)
            buf_.resize(bytes);
    }
    Command c = MakeCommand(dev_->BigEndian(), MODULE_OSC, 0xff, 0x03, (uint32_t)(bytes >> 1));
    c.ask = Ask::DataIn;
    c.data = buf_.data();
    c.data_len = bytes;
    dev_->AddCmd(std::move(c));
}

// Model specific range commands -------------------------------------------

void OscCard::SendInputZoomCtrlCmd(unsigned c)
{
    OscChannelState& s = ch_[c];
    const OscChannelDesc& d = dev_->info.osc.ch[c];
    int jz = (int)d.Jiaozhun(s.inzoom, s.zoom);
    Payload p(dev_->BigEndian());
    p.u16((uint16_t)jz);
    // most models get the opposite setting once first so the relay really switches
    bool toggle = model_ != OSC_MSO21PLUS && model_ != OSC_MSO41;
    if (toggle && !s.inzoom_sent) {
        s.inzoom_sent = true;
        Cmd((uint8_t)(c + 1), 0x06, s.inzoom == 0, &p);
    }
    Cmd((uint8_t)(c + 1), 0x06, s.inzoom, &p);
}

void OscCard::SendZoomCtrlCmd(unsigned c)
{
    OscChannelState& s = ch_[c];
    const OscChannelDesc& d = dev_->info.osc.ch[c];
    int16_t dac = s.zoom < d.zoom.size() ? d.zoom[s.zoom].dac : 0;
    // DAC reference voltage (mV) and optional third word per model
    double vref = 3300.0;
    int extra = -1;
    switch (model_) {
    case OSC_MSO21: vref = 4250.0; break;
    case OSC_MSO21V2:
    case OSC_MSO21V2_2:
    case OSC_MSO21PLUS: extra = 0x060f; break;
    case OSC_MSO41:
        vref = 2500.0;
        extra = 0x07ff;
        break;
    default: break;
    }
    uint16_t a = (uint16_t)(int)((((double)dac + 1250.0) * 0.5 * 4096.0) / vref);
    uint16_t jz = (uint16_t)(int)d.Jiaozhun(s.inzoom, s.zoom);
    Payload p(dev_->BigEndian());
    p.u16(a).u16(jz);
    if (extra >= 0)
        p.u16((uint16_t)extra);
    Cmd((uint8_t)(c + 1), 0x04, s.zoom, &p);
}

void OscCard::SendAcDcCmd(unsigned c)
{
    OscChannelState& s = ch_[c];
    bool toggle = model_ == OSC_MSO10 || model_ == OSC_MSO20 || model_ == OSC_MSO21 || model_ == OSC_MSO21V2;
    if (toggle && !s.acdc_sent) {
        s.acdc_sent = true;
        Payload p0(dev_->BigEndian());
        p0.u8((uint8_t)(1 - (s.ac ? 1 : 0)));
        Cmd((uint8_t)(c + 1), 0x19, 1, &p0);
    }
    Payload p(dev_->BigEndian());
    p.u8(s.ac ? 1 : 0);
    Cmd((uint8_t)(c + 1), 0x19, 1, &p);
}

// ------------------------------------------------------------------ start

void OscCard::Start()
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (started_)
        return;
    const OscDesc& o = dev_->info.osc;
    unsigned nch = o.ChannelNum();
    dev_->RegisterModule(MODULE_OSC, this);
    dev_->SetDataSink(this);
    dev_->DestroyAllCmd(MODULE_OSC);
    SendResetCmd();
    if (o.adc_adjust_num == 1 || o.adc_adjust_num == 2) {
        for (unsigned c = 0; c < nch; c++) {
            const auto& adj = o.ch[c].adc_adjust;
            if (o.adc_adjust_num == 1)
                SendAdcAdjustCmd((uint8_t)(c + 1), adj.size() > 0 ? adj[0] : 0, 0);
            else
                SendAdcAdjustCmd((uint8_t)(c + 1), adj.size() > 0 ? adj[0] : 0, adj.size() > 1 ? adj[1] : 0);
        }
    }
    SendLogicChannelStopCmd();
    for (unsigned c = 0; c < nch; c++)
        SendStartChannelCmd((uint8_t)(c + 1));
    chmask_ = (uint16_t)((1u << nch) - 1);
    SendStreamModeCmd(0);
    sample_idx_ = o.sample_rates.empty() ? 0 : (unsigned)o.sample_rates.size() - 1;
    SendSampleCtrlCmd(sample_idx_);
    for (unsigned c = 0; c < nch; c++) {
        ch_[c].inzoom_sent = false;
        ch_[c].inzoom = (unsigned)o.ch[c].inzoom.size() - 1;
        SendInputZoomCtrlCmd(c);
    }
    for (unsigned c = 0; c < nch; c++) {
        ch_[c].zoom = o.ch[c].zoom.empty() ? 0 : (unsigned)o.ch[c].zoom.size() - 1;
        if (!o.ch[c].zoom.empty())
            SendZoomCtrlCmd(c);
    }
    if (o.support_acdc)
        for (unsigned c = 0; c < nch; c++) {
            ch_[c].acdc_sent = false;
            SendAcDcCmd(c);
        }
    if (hard_trigger_ == 1) {
        SendChufaModeCmd(trig_mode_);
        SendChufaStyleCmd(trig_src_, trig_style_);
        SendChufaSourceCmd(trig_src_, trig_src_, trig_style_);
        if (o.support_trigger_level == 1)
            SendTriggerLevelCmd(0x13, level_v_, sense_v_, trig_src_);
        if (IsSupportPulseWidth())
            SendTriggerPulseNsCmd(pulse_down_ns_, pulse_up_ns_);
    }
    started_ = true;
}

void OscCard::Stop()
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (!started_)
        return;
    started_ = false;
    for (unsigned c = 0; c < ch_.size(); c++)
        SendStopChannelCmd((uint8_t)(c + 1));
    SendLogicChannelStopCmd();
}

// ---------------------------------------------------------- configuration

void OscCard::TriggerSenseUpdate(double span_mv)
{
    double s = ((span_mv / 1000.0) / 10.0) * sense_div_;
    if (s == sense_v_)
        return;
    sense_v_ = s;
    if (hard_trigger_ != 1)
        return;
    if (dev_->info.osc.support_trigger_level == 1)
        SendTriggerLevelCmd(0x24, level_v_, sense_v_, trig_src_);
}

void OscCard::SetRangemV(unsigned c, int min_mv, int max_mv)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    range_min_mv_[c] = (double)min_mv;
    range_max_mv_[c] = (double)max_mv;
    TriggerSenseUpdate((double)(max_mv - min_mv));
    double vmax = (double)max_mv / 1000.0;
    double vmin = (double)min_mv / 1000.0;
    ch_[c].inzoom = InputZoomFor(c, vmin, vmax);
    ch_[c].zoom = ZoomFor(c, ch_[c].inzoom, vmin, vmax);
    SendInputZoomCtrlCmd(c);
    SendZoomCtrlCmd(c);
    if (trig_src_ == (1ull << c))
        SendTriggerLevelCmd(0x13, level_v_, sense_v_, trig_src_);
}

unsigned OscCard::GetSample() const
{
    const auto& r = dev_->info.osc.sample_rates;
    return sample_idx_ < r.size() ? r[sample_idx_] : 0;
}

unsigned OscCard::SetSample(unsigned rate)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    sample_idx_ = Sample2Index(rate);
    SendSampleCtrlCmd(sample_idx_);
    return GetSample();
}

void OscCard::SetTriggerMode(unsigned mode)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    trig_mode_ = mode;
    if (started_ && hard_trigger_ == 1)
        SendChufaModeCmd(mode);
}

void OscCard::SetTriggerStyle(unsigned style)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    trig_style_ = style;
    if (started_ && hard_trigger_ == 1)
        SendChufaStyleCmd(trig_src_, style);
}

void OscCard::SetTriggerSource(uint64_t src)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    uint64_t old = trig_src_;
    trig_src_ = src;
    if (started_ && hard_trigger_ == 1) {
        SendChufaSourceCmd(old, src, trig_style_);
        SendTriggerLevelCmd(0x13, level_v_, sense_v_, src);
    }
}

void OscCard::SetTriggerLevel(double level_v, double sense_v)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    level_v_ = level_v;
    sense_v_ = sense_v;
    if (!(started_ && hard_trigger_ == 1))
        return;
    if (dev_->info.osc.support_trigger_level == 1)
        SendTriggerLevelCmd(0x24, level_v_, sense_v_, trig_src_);
}

bool OscCard::IsSupportFrontPercent() const
{
    return hard_trigger_ == 1 ? dev_->info.osc.support_front_percent == 1 : true;
}

bool OscCard::IsSupportForce() const
{
    return hard_trigger_ == 1 && dev_->info.osc.support_force == 1;
}

void OscCard::TriggerForce()
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (started_ && IsSupportForce())
        Cmd(0xff, 0x25, 1);
}

bool OscCard::IsSupportPulseWidth() const
{
    return hard_trigger_ == 1 && ((dev_->info.osc.trigger_styles >> 3) & 1);
}

void OscCard::SetPulseWidthNs(int down_ns, int up_ns)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    pulse_down_ns_ = down_ns;
    pulse_up_ns_ = up_ns;
    if (started_ && IsSupportPulseWidth())
        SendTriggerPulseNsCmd(pulse_down_ns_, pulse_up_ns_);
}

bool OscCard::IsSupportAcDc(unsigned) const { return dev_->info.osc.support_acdc != 0; }

void OscCard::SetAcDc(unsigned c, bool ac)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size())
        return;
    ch_[c].ac = ac;
    if (started_)
        SendAcDcCmd(c);
}

bool OscCard::GetAcDc(unsigned c) const
{
    if (c >= ch_.size())
        return true;
    return ch_[c].ac;
}

// ---------------------------------------------------------------- capture

unsigned OscCard::CalFrontPercent(unsigned long len, int pct, int nch, int align) const
{
    unsigned long g = (unsigned long)(long)((nch << 5) / align);
    unsigned long x = ((unsigned long)(long)pct * len) / 100;
    x = (x / g) * g;
    if (g > x)
        return (unsigned)g;
    unsigned long rem = len - x;
    return (unsigned)(g <= rem ? x : rem);
}

unsigned OscCard::Capture(int length_kb, uint16_t mask, bool force_length)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    abort_ = false;
    const unsigned nch = ChannelNum();
    const uint16_t all = (uint16_t)((1u << nch) - 1);
    if (chmask_ != mask) {
        uint16_t m = mask & (0x100 | all);
        bool lg = (m & 0x100) != 0;
        // with every analog channel on there is no room for the logic data
        if (m == (0x100 | all)) {
            lg = false;
            m = all;
        }
        for (unsigned c = 0; c < nch; c++) {
            if ((m >> c) & 1)
                SendStartChannelCmd((uint8_t)(c + 1));
            else
                SendStopChannelCmd((uint8_t)(c + 1));
        }
        if (lg)
            SendLogicChannelStartCmd(0xffff);
        else
            SendLogicChannelStopCmd();
        chmask_ = m;
    }
    unsigned align = BlockAlign();
    unsigned bytes = BlockBytes();
    if (!align)
        return 0;
    // MSO10: memory is split in 4 blocks whatever the channel selection
    unsigned long depth = model_ == OSC_MSO10 ? HardMemoryDepth() / 4 : HardMemoryDepth() / bytes;
    unsigned n = (unsigned)depth;
    if (!force_length) {
        unsigned rate = GetSample();
        if ((rate / 1000 << 10) <= n)
            n = rate / 1000 << 10;
    }
    if ((int)(length_kb * 0x400) <= (int)n)
        n = (unsigned)(length_kb * 0x400);
    length_ = (long)(int)n;
    if (buf_.size() < (size_t)n * bytes)
        buf_.resize((size_t)n * bytes);
    unsigned front = CalFrontPercent(length_, front_percent_, (int)nch, (int)align);
    const auto& r = dev_->info.osc.sample_rates;
    unsigned maxr = r.empty() ? 1 : r.back();
    unsigned rate = GetSample();
    uint32_t t = rate ? (uint32_t)(((unsigned long)maxr * length_) / rate) : 0;
    if (model_ == OSC_MSO20V4) {
        uint32_t p5 = force_length && rate ? (uint32_t)(length_ / rate) : 0;
        SendCaptureCmd0((uint32_t)length_, front, t, maxr / 10 - 1, p5, 0);
        return (unsigned)(length_ >> 10);
    }
    SendCaptureCmd0((uint32_t)length_, front, t, 0, 0, 0);
    return (unsigned)(length_ >> 10);
}

void OscCard::OnReply(const uint8_t* p, size_t n, bool)
{
    if (n < 4 || p[0] != MODULE_OSC)
        return;
    if (p[2] == 0xff) {
        uint8_t sub = p[3];
        if (sub == 0x10) {
            if (n >= 8) {
                std::lock_guard<std::recursive_mutex> lk(m_);
                trigger_point_ = logic_trigger_point_ = (double)RdBe32(p + 4);
            }
        } else if (sub == 0x11) {
            if (started_) {
                Cmd(0xff, 0x16, 0);
                SleepMs(10);
                if (abort_) {
                    abort_ = false;
                    SendAbortCaptureCmd();
                }
            }
        }
        return;
    }
    switch (p[2]) {
    case 0x02:
        SampleResult(p, n);
        break;
    case 0x01:
        if (p[3] == 1 && p[1] >= 1 && p[1] <= ch_.size())
            ch_[p[1] - 1].started = true;
        break;
    case 0x10:
        if (p[3] == 1 && p[1] >= 1 && p[1] <= ch_.size())
            ch_[p[1] - 1].started = false;
        break;
    default:
        break;
    }
}

void OscCard::SampleResult(const uint8_t* p, size_t n)
{
    uint8_t st = p[3];
    if (st == 1) {
        if (n >= 8) {
            std::lock_guard<std::recursive_mutex> lk(m_);
            trigger_point_ = logic_trigger_point_ = (double)RdBe32(p + 4);
        }
        if (hard_trigger_ == 1)
            SendReadDataCmd();
    } else if (st == 0x10 || st == 0x11) {
        if (!started_)
            return;
        Cmd(0xff, st == 0x10 ? 0x16 : 0x63, 0);
        SleepMs(10);
        if (abort_) {
            abort_ = false;
            SendAbortCaptureCmd();
        }
    }
}

void OscCard::OnData(const uint8_t*, size_t n, bool)
{
    if (n == 0)
        return;
    DataUpdate();
    if (on_data_ready)
        on_data_ready();
}

// Refines the hardware trigger position by looking for the level crossing
// around it and interpolating linearly between the two samples.
void OscCard::DataUpdate()
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    double tp = trigger_point_;
    double res = tp;
    uint64_t src = trig_src_;
    if (!(src - 1 < 8)) {
        logic_trigger_point_ = tp;
        return;
    }
    unsigned style = trig_style_;
    bool rising = true;
    if (style < 0x21)
        rising = !((0x100010114ull >> (style & 0x3f)) & 1);
    int c = TrigSrcToAnalog(src);
    if (c < 0 || !ChannelCaptured(chmask_, (unsigned)c)) {
        trigger_point_ = logic_trigger_point_ = res;
        return;
    }
    int off = CalDataIndex(chmask_, (unsigned)c);
    unsigned n = (unsigned)length_;
    double z = Ad2VoltagePara() * TotalZoom((unsigned)c);
    double b = BiasContrl((unsigned)c);
    double L = level_v_;
    bool anyedge = style == 4 || style == 0x100 || style == 1 || style == 0x40 || style == 0x80;
    auto val = [&](int i) { return (double)SampleAt((unsigned)off, (unsigned)i) * z + b; };
    if (!(tp < (double)n)) {
        trigger_point_ = logic_trigger_point_ = res;
        return;
    }
    int it = (int)tp;
    bool found = false;
    if (it > 1) {
        double lo = tp - 6.0;
        double di = (double)it;
        int i = it;
        while (lo < di) {
            i--;
            double vl = val(i + 1);   // later sample
            double ve = val(i);       // earlier sample
            bool cond;
            if (!rising)
                cond = anyedge ? ((vl <= L && L <= ve) || (L <= vl && ve <= L)) : (vl <= L && L <= ve);
            else
                cond = anyedge ? (L <= vl && ve <= L) : false;
            if (cond) {
                res = di;
                if (ve != vl)
                    res = di - (vl - L) / (vl - ve);
                found = true;
                break;
            }
            if (i == 1)
                break;
            di = (double)i;
        }
    }
    (void)found;
    if (tp == res && (unsigned)(it + 1) < n) {
        double hi = tp + 6.0;
        double di = (double)it;
        unsigned k = (unsigned)it + 1;
        int i = it;
        while (di < hi) {
            double ve = val(i);       // earlier
            double vl = val(i + 1);   // later
            bool cond;
            if (anyedge)
                cond = !rising ? ((ve <= L && L <= vl) || (L <= ve && vl <= L)) : (ve <= L && L <= vl);
            else
                cond = !rising ? (L <= ve && vl <= L) : false;
            if (cond) {
                res = di;
                if (vl != ve)
                    res = (double)(int)k - (vl - L) / (vl - ve);
                break;
            }
            if (n <= k + 1)
                break;
            i++;
            di = (double)(int)k;
            k++;
        }
    }
    trigger_point_ = res;
    logic_trigger_point_ = res;
}

unsigned OscCard::ReadVoltageDatas(unsigned c, double* out, unsigned len)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size() || !ChannelCaptured(chmask_, c))
        return 0;
    int off = CalDataIndex(chmask_, c);
    if (off < 0)
        return 0;
    unsigned long n = len;
    if (length_ <= n)
        n = length_;
    double para = Ad2VoltagePara();
    double zoom = TotalZoom(c);
    double bias = BiasContrl(c);
    if (WordBytes() == 1) {
        unsigned align = BlockAlign();
        const uint8_t* src = buf_.data() + off;
        for (unsigned long i = 0; i < n; i++)
            out[i] = (double)src[i * align] * para * zoom + bias;
    } else {
        for (unsigned long i = 0; i < n; i++)
            out[i] = (double)SampleAt((unsigned)off, (unsigned)i) * para * zoom + bias;
    }
    // the first and last samples of a capture are not valid (not done by the MSO10)
    if (n > 6 && model_ != OSC_MSO10) {
        double v = out[6];
        for (int i = 0; i < 6; i++)
            out[i] = v;
        double e = out[n - 5];
        out[n - 1] = out[n - 2] = out[n - 3] = out[n - 4] = e;
    }
    return (unsigned)n;
}

// 8 bit models return bytes, 12 bit models 16 bit words (buf is then an unsigned short array)
unsigned OscCard::ReadADCDatas(unsigned c, uint8_t* out, unsigned len)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size() || !ChannelCaptured(chmask_, c))
        return 0;
    int off = CalDataIndex(chmask_, c);
    if (off < 0)
        return 0;
    unsigned long n = len;
    if (length_ <= n)
        n = length_;
    bool fix = n > 6 && model_ != OSC_MSO10;
    if (Wide()) {
        uint16_t* o = reinterpret_cast<uint16_t*>(out);
        for (unsigned long i = 0; i < n; i++)
            o[i] = (uint16_t)SampleAt((unsigned)off, (unsigned)i);
        if (fix) {
            uint16_t v = o[6];
            for (int i = 0; i < 6; i++)
                o[i] = v;
            uint16_t e = o[n - 5];
            o[n - 1] = o[n - 2] = o[n - 3] = o[n - 4] = e;
        }
        return (unsigned)n;
    }
    for (unsigned long i = 0; i < n; i++)
        out[i] = (uint8_t)SampleAt((unsigned)off, (unsigned)i);
    if (fix) {
        uint8_t v = out[6];
        for (int i = 0; i < 6; i++)
            out[i] = v;
        uint8_t e = out[n - 5];
        out[n - 1] = out[n - 2] = out[n - 3] = out[n - 4] = e;
    }
    return (unsigned)n;
}

// logic states; one byte per sample (8 channels) or one 16 bit word on 12 bit models
unsigned OscCard::ReadLogicDatas(uint8_t* out, unsigned len)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (!(chmask_ & 0x100))
        return 0;
    unsigned off = AnalogChannelNum(chmask_);
    if (Wide() && WordBytes() == 2) {
        unsigned long n = len >> 1;
        if (length_ < n)
            n = length_;
        uint16_t* o = reinterpret_cast<uint16_t*>(out);
        for (unsigned long i = 0; i < n; i++)
            o[i] = (uint16_t)SampleAt(off, (unsigned)i);
        return (unsigned)n;
    }
    unsigned long n = len;
    if (length_ <= n)
        n = length_;
    for (unsigned long i = 0; i < n; i++)
        out[i] = (uint8_t)SampleAt(off, (unsigned)i);
    return (unsigned)n;
}

int OscCard::IsVoltageDatasOutRange(unsigned c)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    if (c >= ch_.size() || !ChannelCaptured(chmask_, c))
        return -1;
    int off = CalDataIndex(chmask_, c);
    if (off < 0)
        return -1;
    const OscDesc& o = dev_->info.osc;
    double top = std::pow(2.0, (double)o.adc_bits);
    unsigned amax = 0;
    const auto& adj = o.ch[c].adc_adjust;
    if (o.adc_adjust_num && !adj.empty()) {
        amax = adj[0];
        for (auto v : adj)
            if (amax < v)
                amax = v;
    }
    unsigned long last = length_ - 4;
    if (6 < last)
        for (unsigned long i = 6; i < last; i++) {
            unsigned v = SampleAt((unsigned)off, (unsigned)i);
            if ((unsigned)(int)(top - 1.0) == v || amax == v)
                return 1;
        }
    return 0;
}

} // namespace vmopen
