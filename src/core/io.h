// Digital IO lines and auxiliary DAC outputs.
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

#include "timer.h"
#include "usbdev.h"

namespace vmopen {

// IO direction masks
enum IoMask : uint32_t {
    IO_INPUT = 0x01,
    IO_OUTPUT = 0x02,
};

struct IoLine {
    bool when_close_start = false;
    bool started = false;
    uint32_t mask = IO_OUTPUT;
    uint32_t out_state = 0;
    double pulse_hz = 1000.0;
    double pulse_duty = 50.0;
};

struct IoDacState {
    bool power = false;
    int mv = 0;
};

class IoDevice : public ModuleSink {
public:
    static IoDevice* Create(UsbDevice* dev);
    ~IoDevice() override;

    // state read result (0xffffffff on error); runs on the worker thread
    std::function<void(uint32_t)> on_state;

    unsigned IoNumber() const { return (unsigned)io_.size(); }
    unsigned DacNumber() const { return (unsigned)dac_.size(); }

    void SetIOPower(unsigned io, bool en);
    bool GetIOPower(unsigned io) const { return io < io_.size() && io_[io].started; }
    bool SetIOInOutMask(unsigned io, uint32_t mask);
    uint32_t GetIOInOutMask(unsigned io) const { return io < io_.size() ? io_[io].mask : 0; }
    void SetIOOutState(unsigned io, uint32_t st);
    void SetIoPulseData(unsigned io, double hz, double duty);
    double GetIoPulseHz(unsigned io) const { return io < io_.size() ? io_[io].pulse_hz : -1.0; }
    double GetIoPulseRatio(unsigned io) const { return io < io_.size() ? io_[io].pulse_duty : -1.0; }
    void SetIoPulseSyn();
    void SetAutoReadTimeMs(unsigned ms) { read_ms_ = ms; }
    void SendReadIOInStateCmd();

    void SetDacPower(unsigned d, bool en);
    bool GetDacPower(unsigned d) const { return d < dac_.size() && dac_[d].power; }
    void SetDacmV(unsigned d, int mv);
    int GetDacmV(unsigned d) const { return d < dac_.size() ? dac_[d].mv : 0; }

    void OnReply(const uint8_t* p, size_t n, bool ok) override;

private:
    explicit IoDevice(UsbDevice* dev);
    bool Start();
    void Stop();
    void StopIo(unsigned io);
    void Cmd(uint8_t ch, uint8_t cmd, uint32_t value, const Payload* p = nullptr);
    void SendSetIoCtrlCmd(bool en);
    void SendSetIoInOutCmd();
    void SendSetIoStateCmd(unsigned io, uint32_t st);
    void SendSetIoPulseData(unsigned io, double hz, double duty);
    unsigned PowerNum() const;
    bool ReadTimer();

    UsbDevice* dev_;
    mutable std::recursive_mutex m_;
    std::vector<IoLine> io_;
    std::vector<IoDacState> dac_;
    bool started_ = false;
    unsigned read_ms_ = 500;
    Timer read_timer_;
};

} // namespace vmopen
