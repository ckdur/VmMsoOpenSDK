// Single device C API (VmMsoLib.h, libvmmso).
#define VMMSO_LIBRARY
#include "VmMsoLib.h"

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

#define LOCK Context::Lock lk(Ctx().m)
#define ENTRY(ret)                \
    LOCK;                         \
    DevEntry* e = Ctx().First();  \
    if (!e)                       \
        return ret;
#define OSC(ret)                  \
    ENTRY(ret)                    \
    OscCard* o = e->osc.get();    \
    if (!o) {                     \
        LOG("Error: Device not support %s\n", __func__); \
        return ret;               \
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
    c.on_add = [](DevEntry&) {
        if (g_add)
            g_add(g_notice_para);
    };
    c.on_remove = [](DevEntry&) {
        if (g_remove)
            g_remove(g_notice_para);
    };
    c.on_data = [](DevEntry&) {
        if (g_data)
            g_data(g_data_para);
    };
    c.on_io = [](DevEntry&, uint32_t st) {
        if (g_io)
            g_io(g_io_para, st);
    };
    c.on_comm_error = [](DevEntry&, int code) {
        if (g_err)
            g_err(g_err_para, code);
    };
}

} // namespace

DLL_API int WINAPI InitDll(unsigned int en_log, unsigned int en_hard_watchdog)
{
    g_add = nullptr;
    g_remove = nullptr;
    g_data = nullptr;
    g_io = nullptr;
    Ctx().Init(en_log != 0, en_hard_watchdog != 0, true);
    LOCK;
    InstallHooks();
    return 1;
}

DLL_API int WINAPI FinishDll()
{
    g_add = nullptr;
    g_remove = nullptr;
    g_data = nullptr;
    g_io = nullptr;
    Ctx().Finish();
    return 1;
}

DLL_API unsigned int WINAPI GetOnlyId0()
{
    ENTRY(0);
    return e->dev->info.only_id0;
}

DLL_API unsigned int WINAPI GetOnlyId1()
{
    ENTRY(0);
    return e->dev->info.only_id0;
}

DLL_API int WINAPI ScanDevice()
{
    int r = Ctx().Scan();
    LOG("**********ScanDevice %d **********\n", r);
    return r;
}

DLL_API int WINAPI ResetDevice()
{
    ENTRY(0);
    Ctx().ResetDevice(*e);
    return 1;
}

DLL_API int WINAPI CloseDevice()
{
    ENTRY(0);
    Ctx().CloseDevice(*e);
    return 1;
}

DLL_API int WINAPI GetDeviceDesString(char* des, int des_length)
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

DLL_API void WINAPI SetDevCommunicationErrorCallBack(void* ppara, DevCommunicationErrorCallBack callback)
{
    LOCK;
    g_err_para = ppara;
    g_err = callback;
}

DLL_API int WINAPI GetDevCommunicationError()
{
    ENTRY(0);
    return e->comm_error;
}

DLL_API void WINAPI SetDevNoticeCallBack(void* ppara, AddCallBack addcallback, RemoveCallBack rmvcallback)
{
    LOCK;
    g_notice_para = ppara;
    g_add = addcallback;
    g_remove = rmvcallback;
}

DLL_API int WINAPI IsDevAvailable()
{
    ENTRY(0);
    return e->available ? 1 : 0;
}

// ------------------------------------------------------------------ OSC

DLL_API int WINAPI GetOscChannelNum()
{
    ENTRY(0);
    return (int)e->dev->info.osc.ChannelNum();
}

DLL_API int WINAPI GetOscRangeMinmV()
{
    OSC(0);
    return o->Info().osc.range_min_mv;
}

DLL_API int WINAPI GetOscRangeMaxmV()
{
    OSC(0);
    return o->Info().osc.range_max_mv;
}

DLL_API int WINAPI SetOscChannelRangemV(int channel, int minmv, int maxmv)
{
    OSC(0);
    o->SetRangemV((unsigned)channel, minmv, maxmv);
    return 1;
}

DLL_API int WINAPI GetOscSupportSampleRateNum()
{
    OSC(0);
    return (int)o->Info().osc.sample_rates.size();
}

DLL_API int WINAPI GetOscSupportSampleRates(unsigned int* sample, int maxnum)
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

DLL_API unsigned int WINAPI GetOscSampleRate()
{
    OSC(0);
    return o->GetSample();
}

DLL_API unsigned int WINAPI SetOscSampleRate(unsigned int sample)
{
    OSC(0);
    return o->SetSample(sample);
}

DLL_API int WINAPI IsSupportHardTrigger()
{
    OSC(0);
    return o->IsHardTrigger();
}

DLL_API unsigned int WINAPI GetTriggerMode()
{
    OSC(0);
    return o->TriggerMode();
}

DLL_API void WINAPI SetTriggerMode(unsigned int mode)
{
    OSC();
    o->SetTriggerMode(mode);
}

DLL_API unsigned int WINAPI GetTriggerStyle()
{
    OSC(0);
    return o->TriggerStyle();
}

DLL_API void WINAPI SetTriggerStyle(unsigned int style)
{
    OSC();
    o->SetTriggerStyle(style);
}

DLL_API int WINAPI GetTriggerPulseWidthNsMin() { return 500; }
DLL_API int WINAPI GetTriggerPulseWidthNsMax() { return 100000000; }

DLL_API int WINAPI GetTriggerPulseWidthDownNs()
{
    OSC(500);
    return o->PulseDownNs();
}

DLL_API int WINAPI GetTriggerPulseWidthUpNs()
{
    OSC(100000000);
    return o->PulseUpNs();
}

DLL_API void WINAPI SetTriggerPulseWidthNs(int down_ns, int up_ns)
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

DLL_API unsigned int WINAPI GetTriggerSource()
{
    OSC(0xffffffff);
    return TriggerSourceToApi(o->TriggerSource());
}

DLL_API void WINAPI SetTriggerSource(unsigned int source)
{
    OSC();
    o->SetTriggerSource(1ull << (source & 0x3f));
}

DLL_API void WINAPI SetTriggerLevelmV(int level_mv, int sense_mv)
{
    OSC();
    o->SetTriggerLevel((double)level_mv / 1000.0, (double)sense_mv / 1000.0);
}

DLL_API int WINAPI GetTriggerLevelmV()
{
    OSC(0);
    return (int)(long)(o->TriggerLevel() * 1000.0);
}

DLL_API int WINAPI GetTriggerSensemV()
{
    OSC(0);
    return (int)(o->TriggerSense() * 1000.0);
}

DLL_API bool WINAPI IsSupportPreTriggerPercent()
{
    OSC(false);
    return o->IsSupportFrontPercent();
}

DLL_API int WINAPI GetPreTriggerPercent()
{
    OSC(50);
    return o->FrontPercent();
}

DLL_API void WINAPI SetPreTriggerPercent(int front)
{
    OSC();
    if (front < 1)
        front = 1;
    if (front > 99)
        front = 99;
    o->SetFrontPercent(front);
}

DLL_API int WINAPI IsSupportTriggerForce()
{
    OSC(0);
    return o->IsSupportForce();
}

DLL_API void WINAPI TriggerForce()
{
    OSC();
    o->TriggerForce();
}

DLL_API int WINAPI IsSupportAcDc(unsigned int channel)
{
    OSC(0);
    return o->IsSupportAcDc(channel);
}

DLL_API void WINAPI SetAcDc(unsigned int channel, int ac)
{
    OSC();
    o->SetAcDc(channel, ac != 0);
}

DLL_API int WINAPI GetAcDc(unsigned int channel)
{
    OSC(0);
    return o->GetAcDc(channel);
}

DLL_API int WINAPI IsSupportRollMode()
{
    OSC(0);
    return o->IsSupportRollMode();
}

DLL_API int WINAPI SetRollMode(unsigned int)
{
    OSC(0);
    return 0;
}

DLL_API unsigned int WINAPI GetMemoryLength()
{
    OSC(0);
    return o->HardMemoryDepth() >> 10;
}

DLL_API int WINAPI Capture(int length, unsigned short capture_channel, char force_length)
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

DLL_API int WINAPI AbortCapture()
{
    OSC(0);
    o->AbortCapture();
    e->capturing = false;
    return 0;
}

DLL_API void WINAPI SetDataReadyCallBack(void* ppara, DataReadyCallBack datacallback)
{
    LOCK;
    g_data_para = ppara;
    g_data = datacallback;
}

DLL_API int WINAPI IsDataReady()
{
    ENTRY(0);
    return e->data_ready ? 1 : 0;
}

DLL_API unsigned int WINAPI ReadVoltageDatas(char channel, double* buffer, unsigned int length)
{
    OSC(0);
    e->data_ready = false;
    return o->ReadVoltageDatas((unsigned)(unsigned char)channel, buffer, length);
}

DLL_API unsigned int WINAPI ReadADCDatas(char channel, unsigned char* buffer, unsigned int length)
{
    OSC(0);
    e->data_ready = false;
    return o->ReadADCDatas((unsigned)(unsigned char)channel, buffer, length);
}

DLL_API double WINAPI ReadADCToVoltageZoom(char channel)
{
    OSC(0);
    return o->ADCToVoltageZoom((unsigned)(unsigned char)channel);
}

DLL_API double WINAPI ReadADCToVoltageBias(char channel)
{
    OSC(0);
    return o->ADCToVoltageBias((unsigned)(unsigned char)channel);
}

DLL_API double WINAPI ReadVoltageDatasTriggerPoint()
{
    OSC(0);
    e->data_ready = false;
    return o->TriggerPoint();
}

DLL_API int WINAPI IsVoltageDatasOutRange(char channel)
{
    OSC(0);
    return o->IsVoltageDatasOutRange((unsigned)(unsigned char)channel);
}

DLL_API double WINAPI GetVoltageResolution(char channel)
{
    OSC(0);
    return o->VoltageResolution((unsigned)(unsigned char)channel);
}

DLL_API unsigned int WINAPI ReadLogicDatas(unsigned char* buffer, unsigned int length)
{
    OSC(0);
    e->data_ready = false;
    return o->ReadLogicDatas(buffer, length);
}

DLL_API double WINAPI ReadLogicDatasTriggerPoint()
{
    OSC(0);
    e->data_ready = false;
    return o->LogicTriggerPoint();
}

DLL_API void WINAPI SetSelfCmd(int, int) {}

// ------------------------------------------------------------------ DDS

DLL_API int WINAPI IsSupportDDSDevice()
{
    ENTRY(0);
    return (e->dev->info.modules & MOD_DDS) ? 1 : 0;
}

DLL_API int WINAPI GetDDSChannelNum()
{
    DDS(0);
    return (int)d->ChannelNum();
}

DLL_API int WINAPI GetDDSDepth()
{
    DDS(0);
    return (int)d->Depth();
}

DLL_API void WINAPI SetDDSOutMode(unsigned char channel_index, unsigned int out_mode)
{
    DDS();
    d->SetOutMode(channel_index, out_mode);
}

DLL_API unsigned int WINAPI GetDDSOutMode(unsigned char channel_index)
{
    DDS(0);
    return d->GetOutMode(channel_index);
}

DLL_API int WINAPI GetDDSSupportBoxingStyle(int* style)
{
    DDS(0);
    const auto& b = d->Desc().boxing;
    if (style)
        for (size_t i = 0; i < b.size(); i++)
            style[i] = b[i].style;
    return (int)b.size();
}

DLL_API void WINAPI SetDDSBoxingStyle(unsigned char channel_index, unsigned int boxing)
{
    DDS();
    d->SetBoxing(channel_index, boxing);
}

DLL_API unsigned int WINAPI GetDDSBoxingStyle(unsigned char channel_index)
{
    DDS(0);
    return d->GetBoxing(channel_index);
}

DLL_API void WINAPI UpdateDDSArbBuffer(unsigned char channel_index, unsigned short* arb_buffer,
                                       unsigned int arb_buffer_length)
{
    DDS();
    if (arb_buffer_length > d->Depth())
        arb_buffer_length = d->Depth();
    d->UpdateArbBuffer(channel_index, arb_buffer, arb_buffer_length);
}

DLL_API void WINAPI SetDDSFreq(unsigned char channel_index, double freq)
{
    DDS();
    d->SetFreq(channel_index, freq);
}

DLL_API double WINAPI GetDDSFreq(unsigned char channel_index)
{
    DDS(0);
    return d->GetFreq(channel_index);
}

DLL_API void WINAPI SetDDSDutyCycle(unsigned char channel_index, double cycle)
{
    DDS();
    d->SetDutyCycle(channel_index, cycle);
}

DLL_API double WINAPI GetDDSDutyCycle(unsigned char channel_index)
{
    DDS(0);
    return d->GetDutyCycle(channel_index);
}

DLL_API void WINAPI SetDDSPhase(unsigned char channel_index, double phase)
{
    DDS();
    d->SetPhase(channel_index, phase);
}

DLL_API double WINAPI GetDDSPhase(unsigned char channel_index)
{
    DDS(0);
    return d->GetPhase(channel_index);
}

DLL_API int WINAPI GetDDSCurBoxingAmplitudeMv(unsigned int boxing)
{
    DDS(0);
    return d->CurBoxingAmplitudeMv(boxing);
}

DLL_API void WINAPI SetDDSAmplitudeMv(unsigned char channel_index, int amplitdude)
{
    DDS();
    d->SetAmplitudeMv(channel_index, amplitdude);
}

DLL_API int WINAPI GetDDSAmplitudeMv(unsigned char channel_index)
{
    DDS(0);
    return d->GetAmplitudeMv(channel_index);
}

DLL_API int WINAPI GetDDSCurBoxingBiasMvMin(unsigned int boxing)
{
    DDS(0);
    return d->CurBoxingBiasMvMin(boxing);
}

DLL_API int WINAPI GetDDSCurBoxingBiasMvMax(unsigned int boxing)
{
    DDS(0);
    return d->CurBoxingBiasMvMax(boxing);
}

DLL_API void WINAPI SetDDSBiasMv(unsigned char channel_index, int bias)
{
    DDS();
    d->SetBiasMv(channel_index, bias);
}

DLL_API int WINAPI GetDDSBiasMv(unsigned char channel_index)
{
    DDS(0);
    return d->GetBiasMv(channel_index);
}

DLL_API void WINAPI SetDDSSweepStartFreq(unsigned char channel_index, double freq)
{
    DDS();
    d->SetSweepStartFreq(channel_index, freq);
}

DLL_API double WINAPI GetDDSSweepStartFreq(unsigned char channel_index)
{
    DDS(0);
    return d->GetSweepStartFreq(channel_index);
}

DLL_API void WINAPI SetDDSSweepStopFreq(unsigned char channel_index, double freq)
{
    DDS();
    d->SetSweepStopFreq(channel_index, freq);
}

DLL_API double WINAPI GetDDSSweepStopFreq(unsigned char channel_index)
{
    DDS(0);
    return d->GetSweepStopFreq(channel_index);
}

DLL_API void WINAPI SetDDSSweepTime(unsigned char channel_index, unsigned long long int time_ns)
{
    DDS();
    d->SetSweepTime(channel_index, time_ns);
}

DLL_API unsigned long long int WINAPI GetDDSSweepTime(unsigned char channel_index)
{
    DDS(0);
    return d->GetSweepTime(channel_index);
}

DLL_API void WINAPI SetDDSBurstStyle(unsigned char channel_index, int style)
{
    DDS();
    d->SetBurstStyle(channel_index, style);
}

DLL_API int WINAPI GetDDSBurstStyle(unsigned char channel_index)
{
    DDS(0);
    return d->GetBurstStyle(channel_index);
}

DLL_API void WINAPI SetDDSLoopsNum(unsigned char channel_index, unsigned long long int num)
{
    DDS();
    d->SetLoopsNum(channel_index, num);
}

DLL_API unsigned long long int WINAPI GetDDSLoopsNum(unsigned char channel_index)
{
    DDS(0);
    return d->GetLoopsNum(channel_index);
}

DLL_API void WINAPI SetDDSLoopsNumInfinity(unsigned char channel_index, int en)
{
    DDS();
    d->SetLoopsNumInfinity(channel_index, en != 0);
}

DLL_API int WINAPI GetDDSLoopsNumInfinity(unsigned char channel_index)
{
    DDS(0);
    return d->GetLoopsNumInfinity(channel_index);
}

DLL_API void WINAPI SetDDSBurstPeriodNs(unsigned char channel_index, unsigned long long int ns)
{
    DDS();
    d->SetBurstPeriodNs(channel_index, ns);
}

DLL_API unsigned long long int WINAPI GetDDSBurstPeriodNs(unsigned char channel_index)
{
    DDS(0);
    return d->GetBurstPeriodNs(channel_index);
}

DLL_API void WINAPI SetDDSBurstDelayNs(unsigned char channel_index, unsigned long long int ns)
{
    DDS();
    d->SetBurstDelayNs(channel_index, ns);
}

DLL_API unsigned long long int WINAPI GetDDSBurstDelayNs(unsigned char channel_index)
{
    DDS(0);
    return d->GetBurstDelayNs(channel_index);
}

DLL_API void WINAPI SetDDSTriggerSource(unsigned char channel_index, unsigned int src)
{
    DDS();
    d->SetTriggerSource(channel_index, src);
}

DLL_API unsigned int WINAPI GetDDSTriggerSource(unsigned char channel_index)
{
    DDS(0);
    return d->GetTriggerSource(channel_index);
}

DLL_API void WINAPI SetDDSTriggerSourceIo(unsigned char channel_index, unsigned int io)
{
    DDS();
    d->SetTriggerSourceIo(channel_index, io);
}

DLL_API unsigned int WINAPI GetDDSTriggerSourceIo(unsigned char channel_index)
{
    DDS(0);
    return d->GetTriggerSourceIo(channel_index);
}

DLL_API void WINAPI SetDDSTriggerSourceEnge(unsigned char channel_index, unsigned int enge)
{
    DDS();
    d->SetTriggerSourceEnge(channel_index, enge);
}

DLL_API unsigned int WINAPI GetDDSTriggerSourceEnge(unsigned char channel_index)
{
    DDS(0);
    return d->GetTriggerSourceEnge(channel_index);
}

DLL_API void WINAPI SetDDSOutputGateEnge(unsigned char channel_index, unsigned int enge)
{
    DDS();
    d->SetOutputGateEnge(channel_index, enge);
}

DLL_API unsigned int WINAPI GetDDSOutputGateEnge(unsigned char channel_index)
{
    DDS(0);
    return d->GetOutputGateEnge(channel_index);
}

DLL_API void WINAPI DDSManualTrigger(unsigned char channel_index)
{
    DDS();
    d->ManualTrigger(channel_index);
}

DLL_API void WINAPI DDSOutputEnable(unsigned char channel_index, int enable)
{
    DDS();
    if (enable)
        d->ChannelStart(channel_index);
    else
        d->ChannelStop(channel_index);
}

DLL_API int WINAPI IsDDSOutputEnable(unsigned char channel_index)
{
    DDS(0);
    return d->ChannelIsStart(channel_index);
}

DLL_API void WINAPI DDSMultSyn()
{
    DDS();
    d->MultSyn();
}

// ------------------------------------------------------------------- IO

DLL_API int WINAPI IsSupportIODevice()
{
    ENTRY(0);
    return (e->dev->info.modules & MOD_IO) ? 1 : 0;
}

DLL_API int WINAPI GetSupportIoNumber()
{
    IO(0);
    return (int)io->IoNumber();
}

DLL_API void WINAPI SetIOReadStateCallBack(void* ppara, IOReadStateCallBack callback)
{
    LOCK;
    g_io_para = ppara;
    g_io = callback;
}

DLL_API int WINAPI IsIOReadStateReady()
{
    ENTRY(0);
    return e->io_ready ? 1 : 0;
}

DLL_API void WINAPI IOEnable(unsigned char channel, unsigned char enable)
{
    IO();
    io->SetIOPower(channel, enable != 0);
}

DLL_API unsigned char WINAPI IsIOEnable(unsigned char channel)
{
    IO(0);
    return io->GetIOPower(channel);
}

DLL_API void WINAPI SetIOInOut(unsigned char channel, unsigned char inout)
{
    IO();
    io->SetIOInOutMask(channel, inout ? IO_OUTPUT : IO_INPUT);
}

DLL_API unsigned char WINAPI GetIOInOut(unsigned char channel)
{
    IO(0);
    return io->GetIOInOutMask(channel) != IO_INPUT;
}

DLL_API void WINAPI SetIOOutState(unsigned char channel, unsigned char state)
{
    IO();
    io->SetIOOutState(channel, state);
}

DLL_API void WINAPI SendAutoReadIOInTimeMs(unsigned int ms)
{
    IO();
    io->SetAutoReadTimeMs(ms < 100 ? 100 : ms);
}

DLL_API unsigned int WINAPI SendReadIOInStateCmd()
{
    IO(0);
    io->SendReadIOInStateCmd();
    return 1;
}

DLL_API unsigned int WINAPI GetIOInState()
{
    IO(0);
    e->io_ready = false;
    return e->io_state;
}

DLL_API void WINAPI SetIOPulseData(unsigned char channel, double hz, double duty)
{
    IO();
    io->SetIoPulseData(channel, hz, duty);
}

DLL_API double WINAPI GetIOPulseFreq(unsigned char channel)
{
    IO(0);
    return io->GetIoPulseHz(channel);
}

DLL_API double WINAPI GetIOPulseDuty(unsigned char channel)
{
    IO(0);
    return io->GetIoPulseRatio(channel);
}

DLL_API void WINAPI SetIOPulseSyn()
{
    IO();
    io->SetIoPulseSyn();
}

DLL_API void WINAPI DACEnable(unsigned char channel, unsigned char enable)
{
    IO();
    io->SetDacPower(channel, enable != 0);
}

DLL_API unsigned char WINAPI IsDACEnable(unsigned char channel)
{
    IO(0);
    return io->GetDacPower(channel);
}

DLL_API void WINAPI SetDACmV(unsigned char channel, int vol_mv)
{
    IO();
    io->SetDacmV(channel, vol_mv);
}

DLL_API int WINAPI GetDACmV(unsigned char channel_index)
{
    IO(0);
    return io->GetDacmV(channel_index);
}

// ------------------------------------------------------------ measurement

DLL_API int WINAPI CalFreq(double* buffer, unsigned int buffer_length, double voltage_resolution, unsigned int sample)
{
    LOCK;
    return Ctx().freq_measure.Cal(buffer, buffer_length, voltage_resolution, sample);
}

DLL_API double WINAPI GetFreq() { return Ctx().freq_measure.freq.v; }

DLL_API double WINAPI GetPhase()
{
    FreqMeasure& m = Ctx().freq_measure;
    return m.phase_ok ? m.phase : 0.0;
}

DLL_API double WINAPI GetPositiveDuty()
{
    FreqMeasure& m = Ctx().freq_measure;
    return m.pos_duty.ok ? m.pos_duty.v : 0.0;
}

DLL_API double WINAPI GetNegativeDuty()
{
    FreqMeasure& m = Ctx().freq_measure;
    return m.neg_duty.ok ? m.neg_duty.v : 0.0;
}

DLL_API int WINAPI CalFreqAndPhaseDif(double* buffer1, double voltage_resolution1, double* buffer2,
                                      double voltage_resolution2, unsigned int buffer_length, unsigned int sample,
                                      double freq_deviation_threshold)
{
    LOCK;
    return Ctx().phase_measure.Cal(buffer1, voltage_resolution1, buffer2, voltage_resolution2, buffer_length, sample,
                                   freq_deviation_threshold);
}

DLL_API double WINAPI GetFreq1() { return Ctx().phase_measure.freq1.v; }
DLL_API double WINAPI GetFreq2() { return Ctx().phase_measure.freq2.v; }

DLL_API double WINAPI GetPhaseDif()
{
    PhaseDifMeasure& m = Ctx().phase_measure;
    return m.phase_ok ? m.phase_dif : 0.0;
}
