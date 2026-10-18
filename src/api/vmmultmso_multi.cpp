// Multi device C API (VmMultMsoLib.h, libvmmultmso).
// Every device is identified by the dev_id passed to the add callback.
#define VMMULTMSO_LIBRARY
#include "VmMultMsoLib.h"

#include <cstring>

#include "../core/log.h"
#include "context.h"

using namespace vmopen;

namespace {

void* g_notice_para = nullptr;
AddCallBack g_add = nullptr;
RemoveCallBack g_remove = nullptr;
void* g_data_para = nullptr;
DataReadyCallBack g_data = nullptr;
void* g_io_para = nullptr;
IOReadStateCallBack g_io = nullptr;
void* g_err_para = nullptr;
DevCommunicationErrorCallBack g_err = nullptr;
void* g_stream_para = nullptr;
StreamDataReadyCallBack g_stream = nullptr;
void* g_raw_para = nullptr;
StreamRawDataReadyCallBack g_raw = nullptr;

#define LOCK Context::Lock lk(Ctx().m)
#define ENTRY(ret)                              \
    LOCK;                                       \
    DevEntry* e = Ctx().Find((int)dev_id);      \
    if (!e) {                                   \
        LOG("Error: Device id error %s\n", __func__); \
        return ret;                             \
    }
#define OSC(ret)                  \
    ENTRY(ret)                    \
    OscCard* o = e->osc.get();    \
    if (!o) {                     \
        LOG("Error: Device not support %s\n", __func__); \
        return ret;               \
    }
#define STREAM(ret)                  \
    ENTRY(ret)                       \
    StreamWave* s = e->stream.get(); \
    if (!s) {                        \
        LOG("Error: Device not support %s\n", __func__); \
        return ret;                  \
    }
#define DDS(ret)                  \
    ENTRY(ret)                    \
    DdsDevice* d = e->dds.get();  \
    if (!d)                       \
        return ret;
#define IO(ret)                   \
    ENTRY(ret)                    \
    IoDevice* io = e->io.get();   \
    if (!io)                      \
        return ret;

void InstallHooks()
{
    Context& c = Ctx();
    c.on_add = [](DevEntry& e) {
        if (g_add)
            g_add(g_notice_para, (unsigned)e.id);
    };
    c.on_remove = [](DevEntry& e) {
        if (g_remove)
            g_remove(g_notice_para, (unsigned)e.id);
    };
    c.on_data = [](DevEntry& e) {
        if (g_data)
            g_data(g_data_para, (unsigned)e.id);
    };
    c.on_io = [](DevEntry& e, uint32_t st) {
        if (g_io)
            g_io(g_io_para, (unsigned)e.id, st);
    };
    c.on_comm_error = [](DevEntry& e, int code) {
        if (g_err)
            g_err(g_err_para, (unsigned)e.id, code);
    };
    c.on_stream_data = [](DevEntry& e, unsigned ch, const double* b, unsigned n, const StreamBlockInfo& i) {
        if (g_stream)
            g_stream(g_stream_para, (unsigned)e.id, (unsigned char)ch, const_cast<double*>(b), n, i.failed,
                     i.success, i.need_total, i.total, i.memory);
    };
    c.on_stream_raw = [](DevEntry& e, const uint8_t* b, unsigned n, const StreamBlockInfo& i) {
        if (g_raw)
            g_raw(g_raw_para, (unsigned)e.id, const_cast<uint8_t*>(b), n, i.failed, i.success, i.need_total,
                  i.total, i.memory);
    };
}

} // namespace

MULT_DLL_API int WINAPI InitDll(unsigned int en_log, unsigned int en_hard_watchdog)
{
    Ctx().Init(en_log != 0, en_hard_watchdog != 0, false);
    LOCK;
    InstallHooks();
    return 1;
}

MULT_DLL_API int WINAPI FinishDll()
{
    {
        LOCK;
        g_add = nullptr;
        g_remove = nullptr;
        g_data = nullptr;
        g_io = nullptr;
        g_stream = nullptr;
        g_raw = nullptr;
    }
    Ctx().Finish();
    return 1;
}

MULT_DLL_API unsigned int WINAPI GetOnlyId0(unsigned int dev_id)
{
    ENTRY(0);
    return e->dev->info.only_id0;
}

MULT_DLL_API unsigned int WINAPI GetOnlyId1(unsigned int dev_id)
{
    ENTRY(0);
    return e->dev->info.only_id0;
}

MULT_DLL_API int WINAPI ScanDevice()
{
    int r = Ctx().Scan();
    LOG("**********ScanDevice %d **********\n", r);
    return r;
}

MULT_DLL_API int WINAPI ResetDevice(unsigned int dev_id)
{
    ENTRY(0);
    Ctx().ResetDevice(*e);
    return 1;
}

MULT_DLL_API int WINAPI CloseDevice(unsigned int dev_id)
{
    ENTRY(0);
    Ctx().CloseDevice(*e);
    return 1;
}

MULT_DLL_API int WINAPI GetDeviceDesString(unsigned int dev_id, char* des, int des_length)
{
    ENTRY(0);
    if (!des || des_length <= 0)
        return 0;
    size_t n = e->desid.size();
    if ((size_t)des_length < n)
        n = (size_t)des_length;
    std::memcpy(des, e->desid.data(), n);
    if (n < (size_t)des_length)
        des[n] = 0;
    return 1;
}

MULT_DLL_API void WINAPI SetDevCommunicationErrorCallBack(void* ppara, DevCommunicationErrorCallBack callback)
{
    LOCK;
    g_err_para = ppara;
    g_err = callback;
}

MULT_DLL_API int WINAPI GetDevCommunicationError(unsigned int dev_id)
{
    ENTRY(0);
    return e->comm_error;
}

MULT_DLL_API void WINAPI SetDevNoticeCallBack(void* ppara, AddCallBack addcallback, RemoveCallBack rmvcallback)
{
    LOCK;
    g_notice_para = ppara;
    g_add = addcallback;
    g_remove = rmvcallback;
}

MULT_DLL_API int WINAPI IsDevAvailable()
{
    LOCK;
    for (auto& e : Ctx().Entries())
        if (e.available)
            return 1;
    return 0;
}

MULT_DLL_API int WINAPI GetOscAdcBit(unsigned int dev_id)
{
    ENTRY(0);
    return e->dev->info.osc.adc_bits;
}

MULT_DLL_API int WINAPI GetOscCaptureMode(unsigned int dev_id)
{
    ENTRY(0);
    return e->stream_mode ? 1 : 0;
}

MULT_DLL_API int WINAPI SetOscCaptureMode(unsigned int dev_id, int is_stream_mode)
{
    ENTRY(0);
    Ctx().SetCaptureMode(*e, is_stream_mode != 0);
    return 1;
}

// --------------------------------------------------------------- Stream

MULT_DLL_API void WINAPI SetStreamDataReadyCallBack(void* ppara, StreamDataReadyCallBack datacallback)
{
    LOCK;
    g_stream_para = ppara;
    g_stream = datacallback;
}

MULT_DLL_API void WINAPI SetStreamRawDataReadyCallBack(void* ppara, StreamRawDataReadyCallBack datacallback)
{
    LOCK;
    g_raw_para = ppara;
    g_raw = datacallback;
}

MULT_DLL_API double WINAPI ReadStreamADCToVoltageZoom(unsigned int dev_id, char channel)
{
    STREAM(0);
    return s->ADCToVoltageZoom((unsigned)(unsigned char)channel);
}

MULT_DLL_API double WINAPI ReadStreamADCToVoltageBias(unsigned int dev_id, char channel)
{
    STREAM(0);
    return s->ADCToVoltageBias((unsigned)(unsigned char)channel);
}

MULT_DLL_API int WINAPI GetStreamRangeMinmV(unsigned int dev_id)
{
    STREAM(0);
    return e->dev->info.osc.range_min_mv;
}

MULT_DLL_API int WINAPI GetStreamRangeMaxmV(unsigned int dev_id)
{
    STREAM(0);
    return e->dev->info.osc.range_max_mv;
}

MULT_DLL_API int WINAPI SetStreamChannelRangemV(unsigned int dev_id, int channel, int minmv, int maxmv)
{
    STREAM(0);
    s->SetMinRangemV((unsigned)channel, minmv);
    s->SetMaxRangemV((unsigned)channel, maxmv);
    return 1;
}

MULT_DLL_API int WINAPI GetStreamSupportSampleRateNum(unsigned int dev_id)
{
    STREAM(0);
    return (int)s->SupportSamples(nullptr, -1);
}

MULT_DLL_API int WINAPI GetStreamSupportSampleRates(unsigned int dev_id, unsigned int* sample, int maxnum)
{
    STREAM(0);
    return (int)s->SupportSamples(sample, maxnum);
}

MULT_DLL_API int WINAPI IsStreamSupportAcDc(unsigned int dev_id, unsigned int channel)
{
    STREAM(0);
    return s->IsSupportAcDc(channel);
}

MULT_DLL_API void WINAPI SetStreamAcDc(unsigned int dev_id, unsigned int channel, int ac)
{
    STREAM();
    s->SetAcDc(channel, ac != 0);
}

MULT_DLL_API int WINAPI GetStreamAcDc(unsigned int dev_id, unsigned int channel)
{
    STREAM(0);
    return s->GetAcDc(channel);
}

MULT_DLL_API double WINAPI GetStreamVoltageResolution(unsigned int dev_id, char channel)
{
    STREAM(0);
    return s->VoltageResolution((unsigned)(unsigned char)channel);
}

MULT_DLL_API int WINAPI StreamCapture(unsigned int dev_id, unsigned int count_every_1s, unsigned long long int length_kb,
                                      unsigned short capture_channel, unsigned int sample_rate)
{
    STREAM(0);
    return s->Capture(count_every_1s, length_kb, capture_channel, sample_rate);
}

MULT_DLL_API void WINAPI StreamStopCapture(unsigned int dev_id)
{
    STREAM();
    s->Stop();
}

// ------------------------------------------------- DDS large arbitrary

MULT_DLL_API void WINAPI SetBurstLargeClkCount(unsigned int dev_id, unsigned char channel_index, unsigned int clk_div_count)
{
    DDS();
    d->SetBurstLargeClkCount(channel_index, clk_div_count);
}

// Only the MDDS10 family supports the large arbitrary buffer.
MULT_DLL_API int WINAPI UpdateArbLargeVolBuffer(unsigned int dev_id, unsigned char, double*, unsigned int arb_buffer_length)
{
    if (arb_buffer_length & 0x7ff)
        return -1;
    ENTRY(-2);
    return 0;
}

// ------------------------------------------- meter / power (not present)

MULT_DLL_API int WINAPI IsSupportMultMeterDevice(unsigned int dev_id)
{
    ENTRY(0);
    return 0;
}
MULT_DLL_API void WINAPI MultMeterEnable(unsigned int, unsigned char, int) {}
MULT_DLL_API int WINAPI IsMultMeterEnable(unsigned int, unsigned char) { return 0; }
MULT_DLL_API int WINAPI SetMultMeterMode(unsigned int, unsigned char, unsigned int, unsigned int) { return 0; }
MULT_DLL_API void WINAPI SetMultMeterCallBack(void*, MultMeterUpdateCallBack) {}
MULT_DLL_API int WINAPI ReadMultMeterValues(unsigned int, unsigned char, char*, char*, char*) { return 0; }

MULT_DLL_API int WINAPI IsSupportPowerSupplyDevice(unsigned int dev_id)
{
    ENTRY(0);
    return 0;
}
MULT_DLL_API void WINAPI PowerSupplyEnable(unsigned int, unsigned char, int) {}
MULT_DLL_API int WINAPI IsPowerSupplyEnable(unsigned int, unsigned char) { return 0; }
MULT_DLL_API int WINAPI SetPowerSupplyVoltage(unsigned int, unsigned char, int) { return 0; }
MULT_DLL_API int WINAPI GetPowerSupplyVoltage(unsigned int, unsigned char) { return 0; }

// ------------------------------------------------------------------ OSC

MULT_DLL_API int WINAPI GetOscChannelNum(unsigned int dev_id)
{
    ENTRY(0);
    return (int)e->dev->info.osc.ChannelNum();
}

MULT_DLL_API int WINAPI GetOscRangeMinmV(unsigned int dev_id)
{
    OSC(0);
    return o->Info().osc.range_min_mv;
}

MULT_DLL_API int WINAPI GetOscRangeMaxmV(unsigned int dev_id)
{
    OSC(0);
    return o->Info().osc.range_max_mv;
}

MULT_DLL_API int WINAPI SetOscChannelRangemV(unsigned int dev_id, int channel, int minmv, int maxmv)
{
    OSC(0);
    o->SetRangemV((unsigned)channel, minmv, maxmv);
    return 1;
}

MULT_DLL_API int WINAPI GetOscSupportSampleRateNum(unsigned int dev_id)
{
    OSC(0);
    return (int)o->Info().osc.sample_rates.size();
}

MULT_DLL_API int WINAPI GetOscSupportSampleRates(unsigned int dev_id, unsigned int* sample, int maxnum)
{
    OSC(0);
    const auto& r = o->Info().osc.sample_rates;
    int n = (int)r.size();
    if (!sample)
        return n;
    if (maxnum < n)
        n = maxnum;
    for (int i = 0; i < n; i++)
        sample[i] = r[i];
    return n < 0 ? 0 : n;
}

MULT_DLL_API unsigned int WINAPI GetOscSampleRate(unsigned int dev_id)
{
    OSC(0);
    return o->GetSample();
}

MULT_DLL_API unsigned int WINAPI SetOscSampleRate(unsigned int dev_id, unsigned int sample)
{
    OSC(0);
    return o->SetSample(sample);
}

MULT_DLL_API int WINAPI IsSupportHardTrigger(unsigned int dev_id)
{
    OSC(0);
    return o->IsHardTrigger();
}

MULT_DLL_API unsigned int WINAPI GetTriggerMode(unsigned int dev_id)
{
    OSC(0);
    return o->TriggerMode();
}

MULT_DLL_API void WINAPI SetTriggerMode(unsigned int dev_id, unsigned int mode)
{
    OSC();
    o->SetTriggerMode(mode);
}

MULT_DLL_API unsigned int WINAPI GetTriggerStyle(unsigned int dev_id)
{
    OSC(0);
    return o->TriggerStyle();
}

MULT_DLL_API void WINAPI SetTriggerStyle(unsigned int dev_id, unsigned int style)
{
    OSC();
    o->SetTriggerStyle(style);
}

MULT_DLL_API int WINAPI GetTriggerPulseWidthNsMin(unsigned int) { return 500; }
MULT_DLL_API int WINAPI GetTriggerPulseWidthNsMax(unsigned int) { return 100000000; }

MULT_DLL_API int WINAPI GetTriggerPulseWidthDownNs(unsigned int dev_id)
{
    OSC(500);
    return o->PulseDownNs();
}

MULT_DLL_API int WINAPI GetTriggerPulseWidthUpNs(unsigned int dev_id)
{
    OSC(100000000);
    return o->PulseUpNs();
}

MULT_DLL_API void WINAPI SetTriggerPulseWidthNs(unsigned int dev_id, int down_ns, int up_ns)
{
    OSC();
    o->SetPulseWidthNs(down_ns, up_ns);
}

static unsigned TriggerSourceToApi(uint64_t s)
{
    int a = TrigSrcToAnalog(s);
    if (a >= 0)
        return (unsigned)a;
    int d = TrigSrcToDigital(s);
    if (d >= 0)
        return (unsigned)(d + 0x10);
    int t = TrigSrcToOut(s);
    if (t >= 0)
        return (unsigned)(t + 0x30);
    return 0xffffffff;
}

MULT_DLL_API unsigned int WINAPI GetTriggerSource(unsigned int dev_id)
{
    OSC(0xffffffff);
    return TriggerSourceToApi(o->TriggerSource());
}

MULT_DLL_API void WINAPI SetTriggerSource(unsigned int dev_id, unsigned int source)
{
    OSC();
    o->SetTriggerSource(1ull << (source & 0x3f));
}

MULT_DLL_API void WINAPI SetTriggerLevelmV(unsigned int dev_id, int level_mv, int sense_mv)
{
    OSC();
    o->SetTriggerLevel((double)level_mv / 1000.0, (double)sense_mv / 1000.0);
}

MULT_DLL_API int WINAPI GetTriggerLevelmV(unsigned int dev_id)
{
    OSC(0);
    return (int)(long)(o->TriggerLevel() * 1000.0);
}

MULT_DLL_API int WINAPI GetTriggerSensemV(unsigned int dev_id)
{
    OSC(0);
    return (int)(o->TriggerSense() * 1000.0);
}

MULT_DLL_API bool WINAPI IsSupportPreTriggerPercent(unsigned int dev_id)
{
    OSC(false);
    return o->IsSupportFrontPercent();
}

MULT_DLL_API int WINAPI GetPreTriggerPercent(unsigned int dev_id)
{
    OSC(50);
    return o->FrontPercent();
}

MULT_DLL_API void WINAPI SetPreTriggerPercent(unsigned int dev_id, int front)
{
    OSC();
    if (front < 1)
        front = 1;
    if (front > 99)
        front = 99;
    o->SetFrontPercent(front);
}

MULT_DLL_API int WINAPI IsSupportTriggerForce(unsigned int dev_id)
{
    OSC(0);
    return o->IsSupportForce();
}

MULT_DLL_API void WINAPI TriggerForce(unsigned int dev_id)
{
    OSC();
    o->TriggerForce();
}

MULT_DLL_API int WINAPI IsSupportAcDc(unsigned int dev_id, unsigned int channel)
{
    OSC(0);
    return o->IsSupportAcDc(channel);
}

MULT_DLL_API void WINAPI SetAcDc(unsigned int dev_id, unsigned int channel, int ac)
{
    OSC();
    o->SetAcDc(channel, ac != 0);
}

MULT_DLL_API int WINAPI GetAcDc(unsigned int dev_id, unsigned int channel)
{
    OSC(0);
    return o->GetAcDc(channel);
}

MULT_DLL_API int WINAPI IsSupportRollMode(unsigned int dev_id)
{
    OSC(0);
    return o->IsSupportRollMode();
}

MULT_DLL_API int WINAPI SetRollMode(unsigned int dev_id, unsigned int)
{
    OSC(0);
    return 0;
}

MULT_DLL_API unsigned int WINAPI GetMemoryLength(unsigned int dev_id)
{
    OSC(0);
    return o->HardMemoryDepth() >> 10;
}

MULT_DLL_API int WINAPI Capture(unsigned int dev_id, int length, unsigned short capture_channel, char force_length)
{
    OSC(0);
    if (e->capturing)
        return 0;
    e->data_ready = false;
    if (capture_channel == 0)
        capture_channel = 3;
    if (length < 2)
        length = 2;
    int r = (int)o->Capture(length, capture_channel, force_length != 0);
    e->capturing = true;
    return r;
}

MULT_DLL_API int WINAPI AbortCapture(unsigned int dev_id)
{
    OSC(0);
    o->AbortCapture();
    e->capturing = false;
    return 0;
}

MULT_DLL_API void WINAPI SetDataReadyCallBack(void* ppara, DataReadyCallBack datacallback)
{
    LOCK;
    g_data_para = ppara;
    g_data = datacallback;
}

MULT_DLL_API int WINAPI IsDataReady(unsigned int dev_id)
{
    ENTRY(0);
    return e->data_ready ? 1 : 0;
}

MULT_DLL_API unsigned int WINAPI ReadVoltageDatas(unsigned int dev_id, char channel, double* buffer, unsigned int length)
{
    OSC(0);
    e->data_ready = false;
    return o->ReadVoltageDatas((unsigned)(unsigned char)channel, buffer, length);
}

MULT_DLL_API unsigned int WINAPI ReadADCDatas(unsigned int dev_id, char channel, unsigned char* buffer, unsigned int length)
{
    OSC(0);
    e->data_ready = false;
    return o->ReadADCDatas((unsigned)(unsigned char)channel, buffer, length);
}

MULT_DLL_API double WINAPI ReadADCToVoltageZoom(unsigned int dev_id, char channel)
{
    OSC(0);
    return o->ADCToVoltageZoom((unsigned)(unsigned char)channel);
}

MULT_DLL_API double WINAPI ReadADCToVoltageBias(unsigned int dev_id, char channel)
{
    OSC(0);
    return o->ADCToVoltageBias((unsigned)(unsigned char)channel);
}

MULT_DLL_API double WINAPI ReadVoltageDatasTriggerPoint(unsigned int dev_id)
{
    OSC(0);
    e->data_ready = false;
    return o->TriggerPoint();
}

MULT_DLL_API int WINAPI IsVoltageDatasOutRange(unsigned int dev_id, char channel)
{
    OSC(0);
    return o->IsVoltageDatasOutRange((unsigned)(unsigned char)channel);
}

MULT_DLL_API double WINAPI GetVoltageResolution(unsigned int dev_id, char channel)
{
    OSC(0);
    return o->VoltageResolution((unsigned)(unsigned char)channel);
}

MULT_DLL_API unsigned int WINAPI ReadLogicDatas(unsigned int dev_id, unsigned char* buffer, unsigned int length)
{
    OSC(0);
    e->data_ready = false;
    return o->ReadLogicDatas(buffer, length);
}

MULT_DLL_API double WINAPI ReadLogicDatasTriggerPoint(unsigned int dev_id)
{
    OSC(0);
    e->data_ready = false;
    return o->LogicTriggerPoint();
}

MULT_DLL_API void WINAPI SetSelfCmd(unsigned int, int, int) {}

// ------------------------------------------------------------------ DDS

MULT_DLL_API int WINAPI IsSupportDDSDevice(unsigned int dev_id)
{
    ENTRY(0);
    return (e->dev->info.modules & MOD_DDS) ? 1 : 0;
}

MULT_DLL_API int WINAPI GetDDSChannelNum(unsigned int dev_id)
{
    DDS(0);
    return (int)d->ChannelNum();
}

MULT_DLL_API int WINAPI GetDDSDepth(unsigned int dev_id)
{
    DDS(0);
    return (int)d->Depth();
}

MULT_DLL_API void WINAPI SetDDSOutMode(unsigned int dev_id, unsigned char channel_index, unsigned int out_mode)
{
    DDS();
    d->SetOutMode(channel_index, out_mode);
}

MULT_DLL_API unsigned int WINAPI GetDDSOutMode(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetOutMode(channel_index);
}

MULT_DLL_API int WINAPI GetDDSSupportBoxingStyle(unsigned int dev_id, int* style)
{
    DDS(0);
    const auto& b = d->Desc().boxing;
    if (style)
        for (size_t i = 0; i < b.size(); i++)
            style[i] = b[i].style;
    return (int)b.size();
}

MULT_DLL_API void WINAPI SetDDSBoxingStyle(unsigned int dev_id, unsigned char channel_index, unsigned int boxing)
{
    DDS();
    d->SetBoxing(channel_index, boxing);
}

MULT_DLL_API unsigned int WINAPI GetDDSBoxingStyle(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetBoxing(channel_index);
}

MULT_DLL_API void WINAPI UpdateDDSArbBuffer(unsigned int dev_id, unsigned char channel_index, unsigned short* arb_buffer,
                                       unsigned int arb_buffer_length)
{
    DDS();
    if (arb_buffer_length > d->Depth())
        arb_buffer_length = d->Depth();
    d->UpdateArbBuffer(channel_index, arb_buffer, arb_buffer_length);
}

MULT_DLL_API void WINAPI SetDDSFreq(unsigned int dev_id, unsigned char channel_index, double freq)
{
    DDS();
    d->SetFreq(channel_index, freq);
}

MULT_DLL_API double WINAPI GetDDSFreq(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetFreq(channel_index);
}

MULT_DLL_API void WINAPI SetDDSDutyCycle(unsigned int dev_id, unsigned char channel_index, double cycle)
{
    DDS();
    d->SetDutyCycle(channel_index, cycle);
}

MULT_DLL_API double WINAPI GetDDSDutyCycle(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetDutyCycle(channel_index);
}

MULT_DLL_API void WINAPI SetDDSPhase(unsigned int dev_id, unsigned char channel_index, double phase)
{
    DDS();
    d->SetPhase(channel_index, phase);
}

MULT_DLL_API double WINAPI GetDDSPhase(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetPhase(channel_index);
}

MULT_DLL_API int WINAPI GetDDSCurBoxingAmplitudeMv(unsigned int dev_id, unsigned int boxing)
{
    DDS(0);
    return d->CurBoxingAmplitudeMv(boxing);
}

MULT_DLL_API void WINAPI SetDDSAmplitudeMv(unsigned int dev_id, unsigned char channel_index, int amplitdude)
{
    DDS();
    d->SetAmplitudeMv(channel_index, amplitdude);
}

MULT_DLL_API int WINAPI GetDDSAmplitudeMv(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetAmplitudeMv(channel_index);
}

MULT_DLL_API int WINAPI GetDDSCurBoxingBiasMvMin(unsigned int dev_id, unsigned int boxing)
{
    DDS(0);
    return d->CurBoxingBiasMvMin(boxing);
}

MULT_DLL_API int WINAPI GetDDSCurBoxingBiasMvMax(unsigned int dev_id, unsigned int boxing)
{
    DDS(0);
    return d->CurBoxingBiasMvMax(boxing);
}

MULT_DLL_API void WINAPI SetDDSBiasMv(unsigned int dev_id, unsigned char channel_index, int bias)
{
    DDS();
    d->SetBiasMv(channel_index, bias);
}

MULT_DLL_API int WINAPI GetDDSBiasMv(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetBiasMv(channel_index);
}

MULT_DLL_API void WINAPI SetDDSSweepStartFreq(unsigned int dev_id, unsigned char channel_index, double freq)
{
    DDS();
    d->SetSweepStartFreq(channel_index, freq);
}

MULT_DLL_API double WINAPI GetDDSSweepStartFreq(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetSweepStartFreq(channel_index);
}

MULT_DLL_API void WINAPI SetDDSSweepStopFreq(unsigned int dev_id, unsigned char channel_index, double freq)
{
    DDS();
    d->SetSweepStopFreq(channel_index, freq);
}

MULT_DLL_API double WINAPI GetDDSSweepStopFreq(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetSweepStopFreq(channel_index);
}

MULT_DLL_API void WINAPI SetDDSSweepTime(unsigned int dev_id, unsigned char channel_index, unsigned long long int time_ns)
{
    DDS();
    d->SetSweepTime(channel_index, time_ns);
}

MULT_DLL_API unsigned long long int WINAPI GetDDSSweepTime(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetSweepTime(channel_index);
}

MULT_DLL_API void WINAPI SetDDSBurstStyle(unsigned int dev_id, unsigned char channel_index, int style)
{
    DDS();
    d->SetBurstStyle(channel_index, style);
}

MULT_DLL_API int WINAPI GetDDSBurstStyle(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetBurstStyle(channel_index);
}

MULT_DLL_API void WINAPI SetDDSLoopsNum(unsigned int dev_id, unsigned char channel_index, unsigned long long int num)
{
    DDS();
    d->SetLoopsNum(channel_index, num);
}

MULT_DLL_API unsigned long long int WINAPI GetDDSLoopsNum(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetLoopsNum(channel_index);
}

MULT_DLL_API void WINAPI SetDDSLoopsNumInfinity(unsigned int dev_id, unsigned char channel_index, int en)
{
    DDS();
    d->SetLoopsNumInfinity(channel_index, en != 0);
}

MULT_DLL_API int WINAPI GetDDSLoopsNumInfinity(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetLoopsNumInfinity(channel_index);
}

MULT_DLL_API void WINAPI SetDDSBurstPeriodNs(unsigned int dev_id, unsigned char channel_index, unsigned long long int ns)
{
    DDS();
    d->SetBurstPeriodNs(channel_index, ns);
}

MULT_DLL_API unsigned long long int WINAPI GetDDSBurstPeriodNs(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetBurstPeriodNs(channel_index);
}

MULT_DLL_API void WINAPI SetDDSBurstDelayNs(unsigned int dev_id, unsigned char channel_index, unsigned long long int ns)
{
    DDS();
    d->SetBurstDelayNs(channel_index, ns);
}

MULT_DLL_API unsigned long long int WINAPI GetDDSBurstDelayNs(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetBurstDelayNs(channel_index);
}

MULT_DLL_API void WINAPI SetDDSTriggerSource(unsigned int dev_id, unsigned char channel_index, unsigned int src)
{
    DDS();
    d->SetTriggerSource(channel_index, src);
}

MULT_DLL_API unsigned int WINAPI GetDDSTriggerSource(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetTriggerSource(channel_index);
}

MULT_DLL_API void WINAPI SetDDSTriggerSourceIo(unsigned int dev_id, unsigned char channel_index, unsigned int io)
{
    DDS();
    d->SetTriggerSourceIo(channel_index, io);
}

MULT_DLL_API unsigned int WINAPI GetDDSTriggerSourceIo(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetTriggerSourceIo(channel_index);
}

MULT_DLL_API void WINAPI SetDDSTriggerSourceEnge(unsigned int dev_id, unsigned char channel_index, unsigned int enge)
{
    DDS();
    d->SetTriggerSourceEnge(channel_index, enge);
}

MULT_DLL_API unsigned int WINAPI GetDDSTriggerSourceEnge(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetTriggerSourceEnge(channel_index);
}

MULT_DLL_API void WINAPI SetDDSOutputGateEnge(unsigned int dev_id, unsigned char channel_index, unsigned int enge)
{
    DDS();
    d->SetOutputGateEnge(channel_index, enge);
}

MULT_DLL_API unsigned int WINAPI GetDDSOutputGateEnge(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->GetOutputGateEnge(channel_index);
}

MULT_DLL_API void WINAPI DDSManualTrigger(unsigned int dev_id, unsigned char channel_index)
{
    DDS();
    d->ManualTrigger(channel_index);
}

MULT_DLL_API void WINAPI DDSOutputEnable(unsigned int dev_id, unsigned char channel_index, int enable)
{
    DDS();
    if (enable)
        d->ChannelStart(channel_index);
    else
        d->ChannelStop(channel_index);
}

MULT_DLL_API int WINAPI IsDDSOutputEnable(unsigned int dev_id, unsigned char channel_index)
{
    DDS(0);
    return d->ChannelIsStart(channel_index);
}

MULT_DLL_API void WINAPI DDSMultSyn(unsigned int dev_id)
{
    DDS();
    d->MultSyn();
}

// ------------------------------------------------------------------- IO

MULT_DLL_API int WINAPI IsSupportIODevice(unsigned int dev_id)
{
    ENTRY(0);
    return (e->dev->info.modules & MOD_IO) ? 1 : 0;
}

MULT_DLL_API int WINAPI GetSupportIoNumber(unsigned int dev_id)
{
    IO(0);
    return (int)io->IoNumber();
}

MULT_DLL_API void WINAPI SetIOReadStateCallBack(void* ppara, IOReadStateCallBack callback)
{
    LOCK;
    g_io_para = ppara;
    g_io = callback;
}

MULT_DLL_API int WINAPI IsIOReadStateReady(unsigned int dev_id)
{
    ENTRY(0);
    return e->io_ready ? 1 : 0;
}

MULT_DLL_API void WINAPI IOEnable(unsigned int dev_id, unsigned char channel, unsigned char enable)
{
    IO();
    io->SetIOPower(channel, enable != 0);
}

MULT_DLL_API unsigned char WINAPI IsIOEnable(unsigned int dev_id, unsigned char channel)
{
    IO(0);
    return io->GetIOPower(channel);
}

MULT_DLL_API void WINAPI SetIOInOut(unsigned int dev_id, unsigned char channel, unsigned char inout)
{
    IO();
    io->SetIOInOutMask(channel, inout ? IO_OUTPUT : IO_INPUT);
}

MULT_DLL_API unsigned char WINAPI GetIOInOut(unsigned int dev_id, unsigned char channel)
{
    IO(0);
    return io->GetIOInOutMask(channel) != IO_INPUT;
}

MULT_DLL_API void WINAPI SetIOOutState(unsigned int dev_id, unsigned char channel, unsigned char state)
{
    IO();
    io->SetIOOutState(channel, state);
}

MULT_DLL_API void WINAPI SendAutoReadIOInTimeMs(unsigned int dev_id, unsigned int ms)
{
    IO();
    io->SetAutoReadTimeMs(ms < 100 ? 100 : ms);
}

MULT_DLL_API unsigned int WINAPI SendReadIOInStateCmd(unsigned int dev_id)
{
    IO(0);
    io->SendReadIOInStateCmd();
    return 1;
}

MULT_DLL_API unsigned int WINAPI GetIOInState(unsigned int dev_id)
{
    IO(0);
    e->io_ready = false;
    return e->io_state;
}

MULT_DLL_API void WINAPI SetIOPulseData(unsigned int dev_id, unsigned char channel, double hz, double duty)
{
    IO();
    io->SetIoPulseData(channel, hz, duty);
}

MULT_DLL_API double WINAPI GetIOPulseFreq(unsigned int dev_id, unsigned char channel)
{
    IO(0);
    return io->GetIoPulseHz(channel);
}

MULT_DLL_API double WINAPI GetIOPulseDuty(unsigned int dev_id, unsigned char channel)
{
    IO(0);
    return io->GetIoPulseRatio(channel);
}

MULT_DLL_API void WINAPI SetIOPulseSyn(unsigned int dev_id)
{
    IO();
    io->SetIoPulseSyn();
}

MULT_DLL_API void WINAPI DACEnable(unsigned int dev_id, unsigned char channel, unsigned char enable)
{
    IO();
    io->SetDacPower(channel, enable != 0);
}

MULT_DLL_API unsigned char WINAPI IsDACEnable(unsigned int dev_id, unsigned char channel)
{
    IO(0);
    return io->GetDacPower(channel);
}

MULT_DLL_API void WINAPI SetDACmV(unsigned int dev_id, unsigned char channel, int vol_mv)
{
    IO();
    io->SetDacmV(channel, vol_mv);
}

MULT_DLL_API int WINAPI GetDACmV(unsigned int dev_id, unsigned char channel_index)
{
    IO(0);
    return io->GetDacmV(channel_index);
}

// ------------------------------------------------------------ measurement

MULT_DLL_API int WINAPI CalFreq(double* buffer, unsigned int buffer_length, double voltage_resolution, unsigned int sample)
{
    LOCK;
    return Ctx().freq_measure.Cal(buffer, buffer_length, voltage_resolution, sample);
}

MULT_DLL_API double WINAPI GetFreq() { return Ctx().freq_measure.freq.v; }

MULT_DLL_API double WINAPI GetPhase()
{
    FreqMeasure& m = Ctx().freq_measure;
    return m.phase_ok ? m.phase : 0.0;
}

MULT_DLL_API double WINAPI GetPositiveDuty()
{
    FreqMeasure& m = Ctx().freq_measure;
    return m.pos_duty.ok ? m.pos_duty.v : 0.0;
}

MULT_DLL_API double WINAPI GetNegativeDuty()
{
    FreqMeasure& m = Ctx().freq_measure;
    return m.neg_duty.ok ? m.neg_duty.v : 0.0;
}

MULT_DLL_API int WINAPI CalFreqAndPhaseDif(double* buffer1, double voltage_resolution1, double* buffer2,
                                      double voltage_resolution2, unsigned int buffer_length, unsigned int sample,
                                      double freq_deviation_threshold)
{
    LOCK;
    return Ctx().phase_measure.Cal(buffer1, voltage_resolution1, buffer2, voltage_resolution2, buffer_length, sample,
                                   freq_deviation_threshold);
}

MULT_DLL_API double WINAPI GetFreq1() { return Ctx().phase_measure.freq1.v; }
MULT_DLL_API double WINAPI GetFreq2() { return Ctx().phase_measure.freq2.v; }

MULT_DLL_API double WINAPI GetPhaseDif()
{
    PhaseDifMeasure& m = Ctx().phase_measure;
    return m.phase_ok ? m.phase_dif : 0.0;
}
