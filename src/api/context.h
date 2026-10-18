// State shared by the C APIs: device list, user callbacks and the API lock.
#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>

#include "../core/analysis.h"
#include "../core/dds.h"
#include "../core/io.h"
#include "../core/osc.h"
#include "../core/stream.h"
#include "../core/usbmng.h"

namespace vmopen {

struct DevEntry {
    int id = 0;
    std::string desid;
    std::shared_ptr<UsbDevice> dev;
    bool available = false;
    std::atomic<bool> data_ready{false};
    std::atomic<bool> capturing{false};
    std::atomic<bool> io_ready{false};
    std::atomic<int> comm_error{0};
    std::atomic<uint32_t> io_state{0};
    bool stream_mode = false;
    std::unique_ptr<OscCard> osc;
    std::unique_ptr<StreamWave> stream;
    std::unique_ptr<IoDevice> io;
    std::unique_ptr<DdsDevice> dds;
};

class Context {
public:
    using Lock = std::unique_lock<std::recursive_timed_mutex>;

    // Hooks set by the API flavour (single or multi device)
    std::function<void(DevEntry&)> on_add;
    std::function<void(DevEntry&)> on_remove;
    std::function<void(DevEntry&)> on_data;
    std::function<void(DevEntry&, int)> on_comm_error;
    std::function<void(DevEntry&, uint32_t)> on_io;
    std::function<void(DevEntry&, unsigned, const double*, unsigned, const StreamBlockInfo&)> on_stream_data;
    std::function<void(DevEntry&, const uint8_t*, unsigned, const StreamBlockInfo&)> on_stream_raw;

    bool Init(bool log, bool watchdog, bool single);
    void Finish();
    int Scan();
    bool Active() const { return mng_ != nullptr; }

    DevEntry* Find(int id);
    DevEntry* First();
    std::list<DevEntry>& Entries() { return devs_; }

    // create/destroy function modules of a device
    bool CreateModules(DevEntry& e, bool osc_too = true);
    void DestroyModules(DevEntry& e);
    void SetCaptureMode(DevEntry& e, bool stream);
    void CloseDevice(DevEntry& e);
    void ResetDevice(DevEntry& e);

    std::recursive_timed_mutex m;
    FreqMeasure freq_measure;
    PhaseDifMeasure phase_measure;

private:
    void Notice(int notice, UsbDevice* d);
    std::unique_ptr<UsbManager> mng_;
    std::list<DevEntry> devs_;
    bool single_ = false;
    std::atomic<bool> finishing_{false};
};

Context& Ctx();

// timed lock used from device threads (like the vendor library: 500 ms)
inline Context::Lock TimedLock(std::recursive_timed_mutex& m)
{
    Context::Lock lk(m, std::defer_lock);
    (void)lk.try_lock_for(std::chrono::milliseconds(500));
    return lk;
}

} // namespace vmopen
