// Stream ("recorder") mode: continuous acquisition in consecutive blocks.
#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "osc.h"
#include "usbdev.h"

namespace vmopen {

struct StreamBlockInfo {
    unsigned failed = 0;
    unsigned success = 0;
    uint64_t need_total = 0;   // samples requested per channel
    uint64_t total = 0;        // samples received so far per channel
    uint64_t memory = 0;       // bytes queued for processing
};

class StreamWave : public ModuleSink {
public:
    static StreamWave* Create(UsbDevice* dev);
    ~StreamWave() override;

    // Called on the stream data thread for every received block.
    std::function<void(unsigned ch, const double* buf, unsigned n, const StreamBlockInfo&)> on_data;
    std::function<void(const uint8_t* buf, unsigned bytes, const StreamBlockInfo&)> on_raw;

    unsigned ChannelNum() const { return (unsigned)ch_.size(); }
    void SetMinRangemV(unsigned c, int mv);
    void SetMaxRangemV(unsigned c, int mv);
    bool IsSupportAcDc(unsigned) const { return dev_->info.osc.support_acdc != 0; }
    void SetAcDc(unsigned c, bool ac);
    bool GetAcDc(unsigned c) const { return c < ch_.size() ? ch_[c].ac : true; }
    unsigned SupportSampleNum() const { return max_index_ + 1; }
    unsigned SupportSamples(unsigned* out, int n) const;

    double Ad2VoltagePara() const;
    double TotalZoom(unsigned c) const;
    double ADCToVoltageZoom(unsigned c) const;
    double ADCToVoltageBias(unsigned c) const;
    double VoltageResolution(unsigned c) const { return ADCToVoltageZoom(c); }

    // returns 0 on success, -1 if the sample rate is not supported in stream mode
    int Capture(unsigned count_every_1s, uint64_t length_kb, uint16_t chmask, unsigned rate);
    void Stop() { running_ = false; }
    bool Running() const { return running_; }

    void OnReply(const uint8_t* p, size_t n, bool ok) override;
    void OnData(const uint8_t* p, size_t n, bool ok) override;

private:
    struct Chan {
        bool checked = false;
        bool ac = false;
        bool acdc_sent = false;
        bool inzoom_sent = false;
        unsigned inzoom = 0, zoom = 0;
        int min_mv = 0, max_mv = 0;
        Probe probe;
    };
    struct Block {
        std::vector<uint8_t> data;
        uint64_t index = 0;
        size_t size = 0;
    };
    enum { ST_RUN = 0, ST_DONE = 1, ST_ERROR = 3, ST_OVERFLOW = 4, ST_WAIT = 5 };

    explicit StreamWave(UsbDevice* dev);
    void Cmd(uint8_t ch, uint8_t cmd, uint32_t value, const Payload* p = nullptr);
    unsigned BytesPerSample() const { return dev_->info.osc.adc_bits == 12 ? 2 : 1; }
    unsigned CheckedNum() const;
    unsigned BlockAlign() const;
    void FillSamples();
    bool Start();
    void SendCapture();
    void SendReadData();
    void SendInputZoom(unsigned c);
    void SendZoom(unsigned c);
    void SendAcDc(unsigned c);
    void SendStyle(uint64_t src, unsigned style);
    void SendLevel(double level, double sense, uint64_t src);
    int Voltage2Ad(unsigned c, double v) const;
    void StopChannels();
    void DataThread();
    void Notify();

    UsbDevice* dev_;
    mutable std::recursive_mutex m_;
    std::vector<Chan> ch_;
    uint32_t logic_mask_ = 0;
    unsigned max_index_ = 12;      // last supported stream sample rate index
    std::vector<unsigned> rates_;  // rates usable for the current channel selection
    unsigned rate_ = 0;
    unsigned count_per_s_ = 1;
    uint64_t total_ = 0x100000;    // samples to acquire
    uint64_t block_ = 0;           // samples per block
    uint64_t nblocks_ = 0;
    uint64_t sent_blocks_ = 0;
    bool running_ = false;
    int state_ = ST_RUN;
    std::shared_ptr<Block> cur_;

    std::thread th_;
    std::mutex qm_;
    std::condition_variable qcv_;
    std::deque<std::shared_ptr<Block>> queue_;
    bool notified_ = false;
    bool exit_ = false;
};

} // namespace vmopen
