// DDS function generator: waveform synthesis is done on the host, the device
// plays a buffer of depth_kb*1024 DAC codes with a phase accumulator.
#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include "usbdev.h"

namespace vmopen {

enum DdsStyle : unsigned {
    BOX_SINE = 0x01,
    BOX_SQUARE = 0x02,
    BOX_RAMP = 0x04,
    BOX_PULSE = 0x08,
    BOX_NOISE = 0x10,
    BOX_DC = 0x20,
    BOX_ARB = 0x40,
};

struct DdsChannel {
    bool started = false;
    std::vector<uint16_t> buf;     // generated DAC codes
    std::vector<uint16_t> arb;     // user arbitrary waveform (0..2^dac_bits-1)
    unsigned style = BOX_SINE;
    double duty = 50.0;
    double freq = 1000.0;
    int ampl_mv = 1000;
    int ampl_corr_mv = 1000;   // amplitude after frequency response correction
    int ampl_sent_mv = 1000;
    int bias_mv = 0;
    double phase = 0.0;
    double delay_ns = 0, rise_ns = 0, fall_ns = 0;
    unsigned out_mode = 0;
    double sweep_start = 1000.0, sweep_stop = 100000.0;
    uint64_t sweep_time_ns = 10000000;
    unsigned trig_src = 0, trig_io = 0, trig_enge = 0, gate_enge = 0;
    uint8_t gate_io_dir = 0;
    int burst_style = 0;
    uint64_t loops = 1;
    bool loops_infinity = false;
    uint64_t burst_period_ns = 10000000;
    uint64_t burst_delay_ns = 0;
    unsigned burst_large_clk = 0;
};

// Pure waveform generator, exposed for tests.
void GenerateDdsBuffer(uint16_t* out, int n, unsigned style, double phase, double duty, double p, double period_ns,
                       double delay_ns, double rise_ns, double fall_ns, short dac_min, short dac_max, int vol_min,
                       int vol_max, bool invert, int ampl, int bias);
void GenerateDdsArbBuffer(uint16_t* out, int n, short dac_min, short dac_max, int vol_min, int vol_max, bool invert,
                          int ampl, int bias, double p, const uint16_t* src);

class DdsDevice : public ModuleSink {
public:
    static DdsDevice* Create(UsbDevice* dev);
    ~DdsDevice() override;

    unsigned ChannelNum() const { return (unsigned)ch_.size(); }
    unsigned Depth() const { return (unsigned)dev_->info.dds.depth_kb << 10; }
    const DdsDesc& Desc() const { return dev_->info.dds; }

    void ChannelStart(unsigned c);
    void ChannelStop(unsigned c);
    bool ChannelIsStart(unsigned c) const { return c < ch_.size() && ch_[c].started; }

    void SetOutMode(unsigned c, unsigned mode);
    unsigned GetOutMode(unsigned c) const { return c < ch_.size() ? ch_[c].out_mode : 0; }
    void SetBoxing(unsigned c, unsigned style);
    unsigned GetBoxing(unsigned c) const { return c < ch_.size() ? ch_[c].style : 1; }
    void UpdateArbBuffer(unsigned c, const uint16_t* buf, unsigned len);
    void SetFreq(unsigned c, double f);
    double GetFreq(unsigned c) const { return c < ch_.size() ? ch_[c].freq : 0; }
    void SetDutyCycle(unsigned c, double d);
    double GetDutyCycle(unsigned c) const { return c < ch_.size() ? ch_[c].duty : 0; }
    void SetPhase(unsigned c, double ph);
    double GetPhase(unsigned c) const { return c < ch_.size() ? ch_[c].phase : 0; }
    void SetAmplitudeMv(unsigned c, int mv);
    int GetAmplitudeMv(unsigned c) const { return c < ch_.size() ? ch_[c].ampl_mv : 0; }
    void SetBiasMv(unsigned c, int mv);
    int GetBiasMv(unsigned c) const { return c < ch_.size() ? ch_[c].bias_mv : 0; }
    int CurBoxingAmplitudeMv(unsigned style) const;
    int CurBoxingBiasMvMin(unsigned style) const;
    int CurBoxingBiasMvMax(unsigned style) const;

    void SetSweepStartFreq(unsigned c, double f);
    double GetSweepStartFreq(unsigned c) const { return c < ch_.size() ? ch_[c].sweep_start : 0; }
    void SetSweepStopFreq(unsigned c, double f);
    double GetSweepStopFreq(unsigned c) const { return c < ch_.size() ? ch_[c].sweep_stop : 0; }
    void SetSweepTime(unsigned c, uint64_t ns);
    uint64_t GetSweepTime(unsigned c) const { return c < ch_.size() ? ch_[c].sweep_time_ns : 0; }

    void SetBurstStyle(unsigned c, int s);
    int GetBurstStyle(unsigned c) const { return c < ch_.size() ? ch_[c].burst_style : 0; }
    void SetLoopsNum(unsigned c, uint64_t n);
    uint64_t GetLoopsNum(unsigned c) const { return c < ch_.size() ? ch_[c].loops : 0; }
    void SetLoopsNumInfinity(unsigned c, bool en);
    bool GetLoopsNumInfinity(unsigned c) const { return c < ch_.size() && ch_[c].loops_infinity; }
    void SetBurstPeriodNs(unsigned c, uint64_t ns);
    uint64_t GetBurstPeriodNs(unsigned c) const { return c < ch_.size() ? ch_[c].burst_period_ns : 0; }
    void SetBurstDelayNs(unsigned c, uint64_t ns);
    uint64_t GetBurstDelayNs(unsigned c) const { return c < ch_.size() ? ch_[c].burst_delay_ns : 0; }
    void SetBurstLargeClkCount(unsigned c, unsigned n);

    void SetTriggerSource(unsigned c, unsigned src);
    unsigned GetTriggerSource(unsigned c) const { return c < ch_.size() ? ch_[c].trig_src : 0; }
    void SetTriggerSourceIo(unsigned c, unsigned io);
    unsigned GetTriggerSourceIo(unsigned c) const { return c < ch_.size() ? ch_[c].trig_io : 0; }
    void SetTriggerSourceEnge(unsigned c, unsigned e);
    unsigned GetTriggerSourceEnge(unsigned c) const { return c < ch_.size() ? ch_[c].trig_enge : 0; }
    void SetOutputGateEnge(unsigned c, unsigned e);
    unsigned GetOutputGateEnge(unsigned c) const { return c < ch_.size() ? ch_[c].gate_enge : 0; }
    void ManualTrigger(unsigned c);
    void MultSyn();

    void OnReply(const uint8_t* p, size_t n, bool ok) override;

private:
    explicit DdsDevice(UsbDevice* dev);
    void Start();
    void Cmd(uint8_t ch, uint8_t cmd, uint32_t value, const Payload* p = nullptr, bool top = false);
    void UpdateBufferCmd(unsigned c);
    void SendFreqCmd(unsigned c, double f);
    void SendEnable(unsigned c, bool en);
    void UpdateSweepInfo(unsigned c);
    void UpdateBurstInfo(unsigned c);
    int CalFengFengZhi(double freq, int mv) const;
    unsigned PowerNum() const;

    UsbDevice* dev_;
    mutable std::recursive_mutex m_;
    std::vector<DdsChannel> ch_;
    bool started_ = false;
};

} // namespace vmopen
