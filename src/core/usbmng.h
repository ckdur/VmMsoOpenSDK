// Device discovery: libusb context, hot plug handling and the list of devices.
#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "usbdev.h"

struct libusb_context;

namespace vmopen {

enum DevNotice {
    NOTICE_ADD = 1,
    NOTICE_REMOVE = 2,
    NOTICE_USING = 4,
};

class UsbManager {
public:
    static const uint16_t kVid = 0xd4a2;
    static const uint16_t kPid0 = 0x5662;
    static const uint16_t kPid1 = 0x5663;

    explicit UsbManager(bool watchdog);
    ~UsbManager();

    // Enumerates the bus and opens every new device. Returns 0 or the last error.
    int ScanDevices();

    // Called with (notice, device) when a device became usable or went away.
    // The callback runs on an internal thread.
    void SetNotice(std::function<void(int, UsbDevice*)> cb) { notice_ = std::move(cb); }

    std::shared_ptr<UsbDevice> FindByDesId(const std::string& id);
    std::vector<std::shared_ptr<UsbDevice>> Devices();
    void CloseDevice(UsbDevice* d);

    // from UsbDevice
    void DeviceStateChanged(UsbDevice* d, uint32_t state);

private:
    struct Pending {
        libusb_device* dev;
        bool arrived;
    };
    int AddDevice(libusb_device* dev);
    void RemoveDevice(libusb_device* dev);
    void EventThread();
    void ActionThread();
    void Post(std::function<void()> fn);
    static int HotplugCb(libusb_context* ctx, libusb_device* dev, int event, void* user);

    libusb_context* ctx_ = nullptr;
    bool watchdog_;
    std::atomic<bool> run_{true};
    std::thread events_;
    std::thread actions_;
    std::mutex am_;
    std::condition_variable acv_;
    std::deque<std::function<void()>> pending_;
    int hp_handles_[2] = {0, 0};
    bool hotplug_ = false;

    std::recursive_mutex m_;
    std::vector<std::shared_ptr<UsbDevice>> devs_;
    std::function<void(int, UsbDevice*)> notice_;
};

} // namespace vmopen
