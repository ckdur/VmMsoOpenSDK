// One connected instrument: USB endpoints, the command worker thread,
// the identification/calibration handshake and reply dispatching.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "devinfo.h"
#include "protocol.h"
#include "timer.h"

struct libusb_device;
struct libusb_device_handle;

namespace vmopen {

class UsbManager;

// Receives replies addressed to one module of the device.
class ModuleSink {
public:
    virtual ~ModuleSink() = default;
    virtual void OnReply(const uint8_t* p, size_t n, bool ok) = 0;
    virtual void OnData(const uint8_t* p, size_t n, bool ok) { (void)p, (void)n, (void)ok; }
};

enum HandshakeState : uint32_t {
    HS_RESET = 0x0,
    HS_FAILED = 0x2,
    HS_CHALLENGE = 0x4,
    HS_RESPONSE = 0x8,
    HS_IDENTIFY = 0x10,
    HS_ADC_ADJUST = 0x20,
    HS_INPUT_ADJUST = 0x40,
    HS_ZOOM_LENGTH = 0x80,
    HS_ZOOM = 0x100,
    HS_DDS_ZOOM = 0x200,
    HS_DDS_DONE = 0x400,
    HS_IO_DAC = 0x800,
    HS_POWER = 0x1000,
    HS_OK = 0x2000,
};

enum CommError {
    COMM_ERROR_CTRLIN = 1,
    COMM_ERROR_CTRLOUT = 2,
    COMM_ERROR_DATAIN = 3,
    COMM_ERROR_DATAOUT = 4,
};

class UsbDevice {
public:
    UsbDevice(UsbManager* mng, libusb_device* dev, uint16_t vid, uint16_t pid, uint8_t bus, uint8_t addr, bool watchdog);
    ~UsbDevice();

    int Open();                // 0 ok
    bool Preflight();
    void Close();        // stop worker, release the interface
    void StartHandShake();

    bool IsOpen() const { return open_; }
    bool HandshakeOk() const { return state_ == HS_OK; }
    uint32_t State() const { return state_; }
    bool Same(libusb_device* d, uint8_t bus, uint8_t addr) const { return d == dev_ || (bus == bus_ && addr == addr_); }
    uint8_t Bus() const { return bus_; }
    uint8_t Addr() const { return addr_; }

    // Identifier used by the vendor library: "<model>(<usb address byte>)"
    std::string DesId() const;

    DevInfo info;
    bool BigEndian() const { return info.BigEndian(); }

    // Commands
    void AddCmd(Command&& c, bool top = false);
    void Cmd(uint8_t mod, uint8_t ch, uint8_t cmd, uint32_t value, const Payload* p = nullptr, bool top = false);
    void DestroyAllCmd(int module);   // -1 = all
    size_t QueueSize();
    bool WaitIdle(unsigned ms);   // wait until all queued commands were executed

    // Reply routing
    void RegisterModule(uint8_t module, ModuleSink* s);
    void UnregisterModule(uint8_t module, ModuleSink* s);
    void SetDataSink(ModuleSink* s);  // receiver of bulk data replies

    // Communication error notification (code = COMM_ERROR_*)
    std::function<void(int)> on_comm_error;
    std::atomic<int> comm_error{0};

private:
    friend class UsbManager;
    bool QueryEndpoints();
    void WorkerThread();
    void CtrlOut(const Command& c);
    void CtrlIn();
    void DatasIn(const Command& c);
    void DatasOut(const Command& c);
    void Fail(int code);
    void OnCtrlReply(const uint8_t* p, size_t n, bool ok);
    void DispatchReply(const uint8_t* p, size_t n, bool ok);
    void DispatchData(const uint8_t* p, size_t n, bool ok);

    // handshake
    void SendResetSerial();
    void HandshakeStep(const uint8_t* p, size_t n);
    void HandshakeSuccess();
    void StaticCmd(uint8_t cmd, uint32_t value, const Payload* p = nullptr, bool top = false);
    bool HandshakeTimer();
    bool HeartTimer();

    UsbManager* mng_;
    libusb_device* dev_;
    libusb_device_handle* h_ = nullptr;
    uint16_t vid_, pid_;
    uint8_t bus_, addr_;
    bool watchdog_;
    uint8_t ep_ctrl_in_ = 0, ep_data_in_ = 0, ep_ctrl_out_ = 0, ep_data_out_ = 0;

    std::atomic<bool> open_{false};
    std::atomic<bool> stop_{false};
    std::atomic<bool> thread_done_{true};
    std::atomic<bool> busy_{false};   // worker is executing a command
    std::thread worker_;
    std::mutex qm_;
    std::condition_variable qcv_;
    std::deque<Command> queue_;

    std::mutex sinks_m_;
    std::vector<std::pair<uint8_t, ModuleSink*>> sinks_;
    ModuleSink* data_sink_ = nullptr;

    // handshake state
    std::atomic<uint32_t> state_{HS_RESET};
    uint8_t challenge_[5] = {0};
    uint8_t response_[5] = {0};
    int hs_ticks_ = 0;
    int hs_failures_ = 0;
    std::vector<uint8_t> zoom_buf_;
    unsigned zoom_stride_ = 0, zoom_chunks_ = 0, zoom_index_ = 0;
    Timer hs_timer_;
    Timer heart_timer_;
    bool heart_started_ = false;
    bool heart_stop_ = false;
};

} // namespace vmopen
