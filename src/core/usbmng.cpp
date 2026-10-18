#include "usbmng.h"

#include <libusb-1.0/libusb.h>

#include <algorithm>

#include "log.h"

namespace vmopen {

UsbManager::UsbManager(bool watchdog) : watchdog_(watchdog)
{
    int r = libusb_init(&ctx_);
    if (r != 0) {
        LOG("libusb_init failed: %s\n", libusb_error_name(r));
        ctx_ = nullptr;
        return;
    }
    actions_ = std::thread(&UsbManager::ActionThread, this);
    if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        hotplug_ = true;
        const uint16_t pids[2] = {kPid0, kPid1};
        for (int i = 0; i < 2; i++) {
            r = libusb_hotplug_register_callback(
                ctx_, (libusb_hotplug_event)(LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT),
                (libusb_hotplug_flag)0, kVid, pids[i], LIBUSB_HOTPLUG_MATCH_ANY,
                [](libusb_context* c, libusb_device* d, libusb_hotplug_event e, void* u) {
                    return HotplugCb(c, d, (int)e, u);
                },
                this, &hp_handles_[i]);
            if (r != 0)
                LOG("Error registering hotplug callback %s\n", libusb_error_name(r));
        }
        events_ = std::thread(&UsbManager::EventThread, this);
    } else {
        LOG("Hotplug capabilities are not supported on this platform\n");
    }
}

UsbManager::~UsbManager()
{
    std::vector<std::shared_ptr<UsbDevice>> devs;
    {
        std::lock_guard<std::recursive_mutex> lk(m_);
        devs.swap(devs_);
    }
    for (auto& d : devs)
        d->Close();
    devs.clear();
    run_ = false;
    {
        std::lock_guard<std::mutex> lk(am_);
        acv_.notify_all();
    }
    if (hotplug_ && ctx_) {
        for (int h : hp_handles_)
            if (h)
                libusb_hotplug_deregister_callback(ctx_, h);
    }
    if (events_.joinable())
        events_.join();
    if (actions_.joinable())
        actions_.join();
    if (ctx_)
        libusb_exit(ctx_);
}

int UsbManager::HotplugCb(libusb_context*, libusb_device* dev, int event, void* user)
{
    UsbManager* self = (UsbManager*)user;
    libusb_ref_device(dev);
    bool arrived = event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED;
    self->Post([self, dev, arrived] {
        if (arrived) {
            uint8_t bus = libusb_get_bus_number(dev), addr = libusb_get_device_address(dev);
            LOG("hotplug arrived %d %d\n", bus, addr);
            self->AddDevice(dev);
        } else {
            self->RemoveDevice(dev);
        }
        libusb_unref_device(dev);
    });
    return 0;
}

void UsbManager::EventThread()
{
    while (run_) {
        timeval tv = {0, 200000};
        int r = libusb_handle_events_timeout(ctx_, &tv);
        if (r != 0 && r != LIBUSB_ERROR_INTERRUPTED)
            LOG("libusb_handle_events_timeout %s\n", libusb_error_name(r));
    }
}

void UsbManager::Post(std::function<void()> fn)
{
    std::lock_guard<std::mutex> lk(am_);
    pending_.push_back(std::move(fn));
    acv_.notify_one();
}

void UsbManager::ActionThread()
{
    for (;;) {
        std::function<void()> fn;
        {
            std::unique_lock<std::mutex> lk(am_);
            acv_.wait(lk, [this] { return !run_ || !pending_.empty(); });
            if (!run_ && pending_.empty())
                return;
            fn = std::move(pending_.front());
            pending_.pop_front();
        }
        if (run_)
            fn();
    }
}

int UsbManager::AddDevice(libusb_device* dev)
{
    if (!ctx_)
        return -1;
    libusb_device_descriptor dd;
    if (libusb_get_device_descriptor(dev, &dd) != 0)
        return -3;
    if (dd.idVendor != kVid || (dd.idProduct != kPid0 && dd.idProduct != kPid1))
        return -3;
    uint8_t bus = libusb_get_bus_number(dev), addr = libusb_get_device_address(dev);
    std::shared_ptr<UsbDevice> d;
    {
        std::lock_guard<std::recursive_mutex> lk(m_);
        for (auto& e : devs_)
            if (e->Same(dev, bus, addr))
                return 0;
        d = std::make_shared<UsbDevice>(this, dev, dd.idVendor, dd.idProduct, bus, addr, watchdog_);
        if (d->Open() != 0)
            return -2;
        devs_.push_back(d);
    }
    LOG("Dev (bus %u, device %u): %04X - %04X\n", bus, addr, dd.idVendor, dd.idProduct);
    SleepMs(100);
    d->StartHandShake();
    return 0;
}

void UsbManager::RemoveDevice(libusb_device* dev)
{
    uint8_t bus = libusb_get_bus_number(dev), addr = libusb_get_device_address(dev);
    std::shared_ptr<UsbDevice> d;
    {
        std::lock_guard<std::recursive_mutex> lk(m_);
        for (auto it = devs_.begin(); it != devs_.end(); ++it)
            if ((*it)->Same(dev, bus, addr)) {
                d = *it;
                break;
            }
    }
    if (!d)
        return;
    LOG("hotplug removed %d %d\n", bus, addr);
    d->Close();
    if (d->HandshakeOk() && notice_)
        notice_(NOTICE_REMOVE, d.get());
    std::lock_guard<std::recursive_mutex> lk(m_);
    devs_.erase(std::remove(devs_.begin(), devs_.end(), d), devs_.end());
}

int UsbManager::ScanDevices()
{
    if (!ctx_)
        return 0;
    libusb_device** list = nullptr;
    ssize_t n = libusb_get_device_list(ctx_, &list);
    int ret = 0;
    for (ssize_t i = 0; i < n; i++) {
        libusb_device_descriptor dd;
        if (libusb_get_device_descriptor(list[i], &dd) != 0)
            continue;
        if (dd.idVendor == kVid && (dd.idProduct == kPid0 || dd.idProduct == kPid1)) {
            int r = AddDevice(list[i]);
            if (r != 0)
                ret = r;
        }
    }
    if (list)
        libusb_free_device_list(list, 1);
    return ret;
}

void UsbManager::DeviceStateChanged(UsbDevice* d, uint32_t state)
{
    Post([this, d, state] {
        std::shared_ptr<UsbDevice> sp;
        {
            std::lock_guard<std::recursive_mutex> lk(m_);
            for (auto& e : devs_)
                if (e.get() == d)
                    sp = e;
        }
        if (!sp)
            return;
        if (state == HS_OK) {
            if (notice_)
                notice_(NOTICE_ADD, sp.get());
        } else if (state == HS_FAILED) {
            sp->Close();
            std::lock_guard<std::recursive_mutex> lk(m_);
            devs_.erase(std::remove(devs_.begin(), devs_.end(), sp), devs_.end());
        }
    });
}

std::shared_ptr<UsbDevice> UsbManager::FindByDesId(const std::string& id)
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    for (auto& e : devs_)
        if (e->HandshakeOk() && e->DesId() == id)
            return e;
    return nullptr;
}

std::vector<std::shared_ptr<UsbDevice>> UsbManager::Devices()
{
    std::lock_guard<std::recursive_mutex> lk(m_);
    return devs_;
}

void UsbManager::CloseDevice(UsbDevice* d)
{
    d->Close();
}

} // namespace vmopen
