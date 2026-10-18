#include "context.h"

#include <ctime>

#include "../core/log.h"

namespace vmopen {

Context& Ctx()
{
    static Context c;
    return c;
}

bool Context::Init(bool log, bool watchdog, bool single)
{
    LogInit(log);
    LOG("InitDll.............. Log=%d Watchdog=%d\n", log, watchdog);
    Lock lk(m);
    if (mng_)
        return true;
    single_ = single;
    finishing_ = false;
    mng_.reset(new UsbManager(watchdog));
    mng_->SetNotice([this](int n, UsbDevice* d) { Notice(n, d); });
    return true;
}

void Context::Finish()
{
    std::unique_ptr<UsbManager> mng;
    std::list<DevEntry> devs;
    {
        Lock lk(m);
        finishing_ = true;
        on_add = nullptr;
        on_remove = nullptr;
        on_data = nullptr;
        on_io = nullptr;
        on_comm_error = nullptr;
        on_stream_data = nullptr;
        on_stream_raw = nullptr;
        devs.swap(devs_);
        mng.swap(mng_);
    }
    for (auto& e : devs) {
        DestroyModules(e);
        if (e.dev)
            e.dev->Close();
    }
    devs.clear();
    mng.reset();
}

int Context::Scan()
{
    UsbManager* mg;
    {
        Lock lk(m);
        mg = mng_.get();
    }
    if (!mg)
        return 0;
    return mg->ScanDevices();
}

DevEntry* Context::Find(int id)
{
    for (auto& e : devs_)
        if (e.id == id)
            return &e;
    return nullptr;
}

DevEntry* Context::First() { return devs_.empty() ? nullptr : &devs_.front(); }

bool Context::CreateModules(DevEntry& e, bool)
{
    UsbDevice* d = e.dev.get();
    DevEntry* ep = &e;
    if (!e.stream_mode) {
        e.osc.reset(OscCard::Create(d));
        if (e.osc) {
            e.osc->on_data_ready = [this, ep] {
                Lock lk = TimedLock(m);
                ep->capturing = false;
                ep->data_ready = true;
                if (on_data)
                    on_data(*ep);
            };
            e.osc->Start();
        }
    } else {
        e.stream.reset(StreamWave::Create(d));
        if (e.stream) {
            e.stream->on_data = [this, ep](unsigned c, const double* b, unsigned n, const StreamBlockInfo& i) {
                if (on_stream_data)
                    on_stream_data(*ep, c, b, n, i);
            };
            e.stream->on_raw = [this, ep](const uint8_t* b, unsigned n, const StreamBlockInfo& i) {
                if (on_stream_raw)
                    on_stream_raw(*ep, b, n, i);
            };
        }
    }
    e.io.reset(IoDevice::Create(d));
    if (e.io) {
        e.io->on_state = [this, ep](uint32_t st) {
            Lock lk = TimedLock(m);
            ep->io_state = st;
            if (on_io)
                on_io(*ep, st);
            ep->io_ready = true;
        };
        for (unsigned i = 0; i < e.io->IoNumber(); i++) {
            e.io->SetIOPower(i, true);
            e.io->SetIOInOutMask(i, IO_OUTPUT);
            e.io->SetIOOutState(i, 0);
        }
    }
    e.dds.reset(DdsDevice::Create(d));
    return e.osc || e.stream || e.dds || e.io;
}

void Context::DestroyModules(DevEntry& e)
{
    e.data_ready = false;
    e.capturing = false;
    e.io_ready = false;
    if (e.osc)
        e.osc->Stop();
    e.osc.reset();
    if (e.stream)
        e.stream->Stop();
    e.stream.reset();
    e.dds.reset();
    e.io.reset();
}

void Context::SetCaptureMode(DevEntry& e, bool stream)
{
    if (e.stream_mode == stream)
        return;
    if (e.osc) {
        e.osc->Stop();
        e.osc.reset();
    }
    if (e.stream) {
        e.stream->Stop();
        e.stream.reset();
    }
    if (e.dev)
        e.dev->WaitIdle(1000);
    e.stream_mode = stream;
    UsbDevice* d = e.dev.get();
    DevEntry* ep = &e;
    if (!stream) {
        e.osc.reset(OscCard::Create(d));
        if (e.osc) {
            e.osc->on_data_ready = [this, ep] {
                Lock lk = TimedLock(m);
                ep->capturing = false;
                ep->data_ready = true;
                if (on_data)
                    on_data(*ep);
            };
            e.osc->Start();
        }
    } else {
        e.stream.reset(StreamWave::Create(d));
        if (e.stream) {
            e.stream->on_data = [this, ep](unsigned c, const double* b, unsigned n, const StreamBlockInfo& i) {
                if (on_stream_data)
                    on_stream_data(*ep, c, b, n, i);
            };
            e.stream->on_raw = [this, ep](const uint8_t* b, unsigned n, const StreamBlockInfo& i) {
                if (on_stream_raw)
                    on_stream_raw(*ep, b, n, i);
            };
        }
    }
}

void Context::CloseDevice(DevEntry& e)
{
    if (e.dev)
        e.dev->Close();
    DestroyModules(e);
}

void Context::ResetDevice(DevEntry& e)
{
    DestroyModules(e);
    SleepMs(1000);
    CreateModules(e);
    SleepMs(1000);
}

void Context::Notice(int notice, UsbDevice* d)
{
    // wait for the API lock without blocking FinishDll forever
    Lock lk(m, std::defer_lock);
    while (!lk.try_lock_for(std::chrono::milliseconds(100)))
        if (finishing_)
            return;
    if (finishing_ || !mng_)
        return;
    std::string id = d->DesId();
    LOG("**********UsbDevAddRemoveCallBack DEVICE_NOTICE_ID=%d %s**********\n", notice, d->info.name.c_str());
    if (notice == NOTICE_ADD) {
        if (single_ && !devs_.empty())
            return;
        for (auto& e : devs_)
            if (e.desid == id)
                return;
        devs_.emplace_back();
        DevEntry& e = devs_.back();
        e.desid = id;
        e.dev = mng_->FindByDesId(id);
        e.id = single_ ? 0 : (int)std::time(nullptr);
        for (auto& o : devs_)
            if (&o != &e && o.id == e.id)
                e.id++;
        DevEntry* ep = &e;
        d->on_comm_error = [this, ep](int code) {
            Lock l2 = TimedLock(m);
            ep->comm_error = code;
            LOG("**********mCommunicationErrorCallBack %d**********\n", code);
            if (on_comm_error)
                on_comm_error(*ep, code);
        };
        bool ok = CreateModules(e);
        if (!ok) {
            devs_.pop_back();
            return;
        }
        e.available = true;
        SleepMs(1000);
        if (on_add)
            on_add(e);
    } else if (notice == NOTICE_REMOVE) {
        for (auto it = devs_.begin(); it != devs_.end(); ++it) {
            if (it->desid != id)
                continue;
            it->available = false;
            if (on_remove)
                on_remove(*it);
            DestroyModules(*it);
            devs_.erase(it);
            break;
        }
    }
}

} // namespace vmopen
