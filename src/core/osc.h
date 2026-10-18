// Oscilloscope (and logic analyser) capture in block mode.
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

#include "usbdev.h"

namespace vmopen {

enum OscModel {
    OSC_MSO10,
    OSC_MSO20,
    OSC_MSO20V4,
    OSC_MSO21,
    OSC_MSO21V2,
    OSC_MSO21V2_2,
    OSC_MSO21PLUS,
    OSC_MSO41,
};

// Trigger source bit masks (1 << TRIGGER_SOURCE_*)
inline int TrigSrcToAnalog(uint64_t s)
{
    return s == 1 ? 0 : s == 2 ? 1 : s == 4 ? 2 : s == 8 ? 3 : -1;
}
inline int TrigSrcToDigital(uint64_t s)
{
    for (int i = 0; i < 32; i++)
        if (s == (1ull << (16 + i)))
            return i;
    return -1;
}
inline int TrigSrcToOut(uint64_t s) { return s == (1ull << 48) ? 0 : -1; }

struct Probe {
    double mul = 1.0;
    double off = 0.0;
};

struct OscChannelState {
    bool acdc_sent = false;
    bool ac = false;
    bool inzoom_sent = false;
    unsigned inzoom = 0;
    unsigned zoom = 0;
    bool invert = false;
    bool started = false;
    Probe probe;
};

class OscCard : public ModuleSink {
public:
    // Returns nullptr if the device has no supported oscilloscope
    static OscCard* Create(UsbDevice* dev);
    virtual ~OscCard();

    std::function<void()> on_data_ready;   // a capture finished (worker thread)

    void Start();
    void Stop();
    bool IsStarted() const { return started_; }

    const DevInfo& Info() const { return dev_->info; }
    unsigned ChannelNum() const { return dev_->info.osc.ChannelNum(); }

    // configuration
    void SetRangemV(unsigned ch, int min_mv, int max_mv);
    unsigned GetSample() const;
    unsigned SetSample(unsigned rate);
    bool IsHardTrigger() const { return hard_trigger_ == 1; }
    unsigned TriggerMode() const { return trig_mode_; }
    void SetTriggerMode(unsigned mode);
    unsigned TriggerStyle() const { return trig_style_; }
    void SetTriggerStyle(unsigned style);
    uint64_t TriggerSource() const { return trig_src_; }
    void SetTriggerSource(uint64_t src_mask);
    void SetTriggerLevel(double level_v, double sense_v);
    double TriggerLevel() const { return level_v_; }
    double TriggerSense() const { return sense_v_; }
    bool IsSupportFrontPercent() const;
    int FrontPercent() const { return front_percent_; }
    void SetFrontPercent(int p) { front_percent_ = IsSupportFrontPercent() ? p : 50; }
    bool IsSupportForce() const;
    void TriggerForce();
    bool IsSupportPulseWidth() const;
    void SetPulseWidthNs(int down_ns, int up_ns);
    int PulseDownNs() const { return pulse_down_ns_; }
    int PulseUpNs() const { return pulse_up_ns_; }
    bool IsSupportAcDc(unsigned ch) const;
    void SetAcDc(unsigned ch, bool ac);
    bool GetAcDc(unsigned ch) const;
    unsigned HardMemoryDepth() const { return dev_->info.osc.memory_kb << 10; }
    bool IsSupportRollMode() const { return false; }

    // capture
    unsigned Capture(int length_kb, uint16_t chmask, bool force_length);
    void AbortCapture() { abort_ = true; }
    unsigned ReadVoltageDatas(unsigned ch, double* buf, unsigned len);
    unsigned ReadADCDatas(unsigned ch, uint8_t* buf, unsigned len);
    unsigned ReadLogicDatas(uint8_t* buf, unsigned len);
    int IsVoltageDatasOutRange(unsigned ch);
    double TriggerPoint() const { return trigger_point_; }
    double LogicTriggerPoint() const { return logic_trigger_point_; }
    double VoltageResolution(unsigned ch) const;
    double ADCToVoltageZoom(unsigned ch) const;
    double ADCToVoltageBias(unsigned ch) const;

    // conversions
    double Ad2VoltagePara() const;
    double TotalZoom(unsigned ch) const;
    double BiasContrl(unsigned ch) const;
    int Voltage2Ad(unsigned ch, double v) const;

    // ModuleSink
    void OnReply(const uint8_t* p, size_t n, bool ok) override;
    void OnData(const uint8_t* p, size_t n, bool ok) override;

protected:
    OscCard(UsbDevice* dev, OscModel model);

    unsigned InputZoomFor(unsigned ch, double min_v, double max_v) const;
    unsigned ZoomFor(unsigned ch, unsigned in, double min_v, double max_v) const;
    unsigned Sample2Index(unsigned rate) const;
    bool Wide() const;                 // 12 bit models: samples are 16 bit words
    unsigned BlockAlign() const;       // words per sample of all captured channels
    unsigned WordBytes() const;        // bytes per word in the transfer buffer
    unsigned BlockBytes() const { return BlockAlign() * WordBytes(); }
    int CalDataIndex(uint16_t mask, unsigned ch) const;
    bool ChannelCaptured(uint16_t mask, unsigned ch) const;
    unsigned AnalogChannelNum(uint16_t mask) const;
    unsigned SampleAt(unsigned ch_off, unsigned i) const;

    // device commands
    void Cmd(uint8_t ch, uint8_t cmd, uint32_t value, const Payload* p = nullptr);
    void SendResetCmd();
    void SendStartChannelCmd(uint8_t ch);
    void SendStopChannelCmd(uint8_t ch);
    void SendLogicChannelStartCmd(uint32_t mask);
    void SendLogicChannelStopCmd();
    void SendStreamModeCmd(int mode);
    void SendSampleCtrlCmd(unsigned idx);
    void SendChufaModeCmd(unsigned mode);
    void SendChufaStyleCmd(uint64_t src, unsigned style);
    void SendChufaSourceCmd(uint64_t old_src, uint64_t new_src, unsigned style);
    void SendTriggerLevelCmd(uint8_t cmd, double level, double sense, uint64_t src);
    void SendTriggerPulseNsCmd(int down, int up);
    void SendAdcAdjustCmd(uint8_t ch, uint32_t a, uint32_t b);
    void SendCaptureCmd0(uint32_t len, uint32_t front, uint32_t time, uint32_t p4, uint32_t p5, unsigned delay);
    void SendReadDataCmd();
    void SendAbortCaptureCmd();
    void TriggerSenseUpdate(double span_mv);

    // per model
    virtual void SendZoomCtrlCmd(unsigned ch);
    virtual void SendInputZoomCtrlCmd(unsigned ch);
    virtual void SendAcDcCmd(unsigned ch);

    void SampleResult(const uint8_t* p, size_t n);
    void DataUpdate();
    unsigned CalFrontPercent(unsigned long len, int pct, int nch, int align) const;

    UsbDevice* dev_;
    OscModel model_;
    mutable std::recursive_mutex m_;
    std::vector<OscChannelState> ch_;
    std::vector<double> range_min_mv_, range_max_mv_;
    unsigned sample_idx_ = 0;
    bool started_ = false;

    // trigger
    int hard_trigger_ = 2;
    unsigned trig_mode_ = 0;
    uint64_t trig_src_ = 1;
    unsigned trig_style_ = 1;
    double level_v_ = 0.0;
    int front_percent_ = 50;
    double sense_div_ = 0.1;
    double sense_v_ = 0.5;
    bool holdoff_ = false;
    unsigned holdoff_ns_ = 10000000;
    int pulse_up_ns_ = 1000;
    int pulse_down_ns_ = 500;

    // capture
    std::vector<uint8_t> buf_;
    uint16_t chmask_ = 0;
    unsigned long length_ = 0;  // samples per channel
    bool abort_ = false;
    double trigger_point_ = 0;
    double logic_trigger_point_ = 0;
};

} // namespace vmopen
