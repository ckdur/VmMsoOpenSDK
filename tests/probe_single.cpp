// Prints every queryable property of the first device and performs captures.
// Built against the single device API (VmMsoLib.h), runs with either the
// vendor libvmmso or the open implementation, so the outputs can be diffed.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
#include "VmMsoLib.h"

static std::atomic<int> added{0}, ready{0};
static void CALLBACK on_add(void*) { added = 1; }
static void CALLBACK on_rmv(void*) { added = 0; }
static void CALLBACK on_data(void*) { ready++; }

static bool wait_for(std::atomic<int>& v, int ms)
{
    for (int i = 0; i < ms / 10 && !v; i++)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return v != 0;
}

static void capture(const char* tag, int len_kb, unsigned short chmask, int nch, int osc_ch)
{
    ready = 0;
    int r = Capture(len_kb, chmask, 1);
    printf("[%s] Capture(%d,%d) -> %d\n", tag, len_kb, chmask, r);
    if (!wait_for(ready, 5000)) {
        printf("[%s] timeout waiting data\n", tag);
        return;
    }
    printf("[%s] IsDataReady=%d TriggerPoint=%f LogicTP=%f\n", tag, IsDataReady(), ReadVoltageDatasTriggerPoint(),
           ReadLogicDatasTriggerPoint());
    std::vector<double> buf(r * 1024 + 16);
    std::vector<unsigned char> adc(r * 1024 + 16);
    for (int c = 0; c < osc_ch; c++) {
        if (!(chmask & (1 << c)))
            continue;
        unsigned n = ReadVoltageDatas(c, buf.data(), buf.size());
        unsigned na = ReadADCDatas(c, adc.data(), adc.size());
        double mn = 1e9, mx = -1e9, avg = 0;
        for (unsigned i = 0; i < n; i++) {
            mn = buf[i] < mn ? buf[i] : mn;
            mx = buf[i] > mx ? buf[i] : mx;
            avg += buf[i];
        }
        if (n)
            avg /= n;
        printf("[%s] ch%d n=%u nadc=%u min=%.4f max=%.4f avg=%.4f zoom=%.6f bias=%.6f res=%.6f outrange=%d\n", tag, c,
               n, na, mn, mx, avg, ReadADCToVoltageZoom(c), ReadADCToVoltageBias(c), GetVoltageResolution(c),
               IsVoltageDatasOutRange(c));
        printf("[%s] ch%d adc[0..16]:", tag, c);
        for (int i = 0; i < 16 && i < (int)na; i++)
            printf(" %02x", adc[i]);
        printf("\n");
        if (CalFreq(buf.data(), n, GetVoltageResolution(c), GetOscSampleRate()))
            printf("[%s] ch%d freq=%.3f phase=%.3f +duty=%.3f -duty=%.3f\n", tag, c, GetFreq(), GetPhase(),
                   GetPositiveDuty(), GetNegativeDuty());
    }
    unsigned nl = ReadLogicDatas(adc.data(), adc.size());
    printf("[%s] logic n=%u first:", tag, nl);
    for (int i = 0; i < 16 && i < (int)nl; i++)
        printf(" %02x", adc[i]);
    printf("\n");
}

int main(int argc, char** argv)
{
    int quick = argc > 1;
    printf("InitDll=%d\n", InitDll(0, 0));
    SetDevNoticeCallBack(nullptr, on_add, on_rmv);
    SetDataReadyCallBack(nullptr, on_data);
    printf("ScanDevice=%d\n", ScanDevice());
    if (!wait_for(added, 5000) && !IsDevAvailable()) {
        printf("no device\n");
        FinishDll();
        return 1;
    }
    printf("IsDevAvailable=%d\n", IsDevAvailable());
    char des[256] = {0};
    printf("GetDeviceDesString=%d '%s'\n", GetDeviceDesString(des, sizeof des), des);
    printf("OnlyId=%08x %08x\n", GetOnlyId0(), GetOnlyId1());
    int nch = GetOscChannelNum();
    printf("OscChannelNum=%d RangeMinmV=%d RangeMaxmV=%d\n", nch, GetOscRangeMinmV(), GetOscRangeMaxmV());
    int ns = GetOscSupportSampleRateNum();
    std::vector<unsigned> rates(ns > 0 ? ns : 1);
    printf("SampleRates(%d):", GetOscSupportSampleRates(rates.data(), ns));
    for (int i = 0; i < ns; i++)
        printf(" %u", rates[i]);
    printf("\nSampleRate=%u Memory=%u KB\n", GetOscSampleRate(), GetMemoryLength());
    printf("HardTrig=%d TrigMode=%u Style=%u Src=%u Level=%d Sense=%d PreTrig=%d(%d) Force=%d\n",
           IsSupportHardTrigger(), GetTriggerMode(), GetTriggerStyle(), GetTriggerSource(), GetTriggerLevelmV(),
           GetTriggerSensemV(), GetPreTriggerPercent(), IsSupportPreTriggerPercent(), IsSupportTriggerForce());
    printf("PulseWidth min=%d max=%d down=%d up=%d\n", GetTriggerPulseWidthNsMin(), GetTriggerPulseWidthNsMax(),
           GetTriggerPulseWidthDownNs(), GetTriggerPulseWidthUpNs());
    for (int c = 0; c < nch; c++)
        printf("ch%d AcDc support=%d val=%d\n", c, IsSupportAcDc(c), GetAcDc(c));
    printf("RollMode=%d\n", IsSupportRollMode());
    printf("DDS=%d ch=%d depth=%d\n", IsSupportDDSDevice(), GetDDSChannelNum(), GetDDSDepth());
    if (IsSupportDDSDevice()) {
        int nst = GetDDSSupportBoxingStyle(nullptr);
        std::vector<int> st(nst > 0 ? nst : 1);
        GetDDSSupportBoxingStyle(st.data());
        printf("DDS styles(%d):", nst);
        for (int i = 0; i < nst; i++)
            printf(" %x", st[i]);
        printf("\n");
        for (int i = 0; i < nst; i++)
            printf("DDS style %x ampl=%d bias=[%d,%d]\n", st[i], GetDDSCurBoxingAmplitudeMv(st[i]),
                   GetDDSCurBoxingBiasMvMin(st[i]), GetDDSCurBoxingBiasMvMax(st[i]));
        for (int c = 0; c < GetDDSChannelNum(); c++)
            printf("DDS%d mode=%u box=%u freq=%f duty=%f phase=%f ampl=%d bias=%d en=%d\n", c, GetDDSOutMode(c),
                   GetDDSBoxingStyle(c), GetDDSFreq(c), GetDDSDutyCycle(c), GetDDSPhase(c), GetDDSAmplitudeMv(c),
                   GetDDSBiasMv(c), IsDDSOutputEnable(c));
    }
    printf("IO=%d num=%d\n", IsSupportIODevice(), GetSupportIoNumber());

    // Configure and capture
    for (int c = 0; c < nch; c++)
        SetOscChannelRangemV(c, -5000, 5000);
    if (ns > 0)
        SetOscSampleRate(rates[ns > 3 ? ns - 4 : 0]);
    printf("SampleRate now=%u\n", GetOscSampleRate());
    SetTriggerMode(TRIGGER_MODE_AUTO);
    SetTriggerStyle(TRIGGER_STYLE_RISE_EDGE);
    SetTriggerSource(TRIGGER_SOURCE_CH1);
    SetTriggerLevelmV(0, 50);
    SetPreTriggerPercent(50);
    if (IsSupportDDSDevice()) {
        SetDDSBoxingStyle(0, BX_SINE);
        SetDDSFreq(0, 1000);
        SetDDSAmplitudeMv(0, 2000);
        SetDDSBiasMv(0, 0);
        SetDDSOutMode(0, DDS_OUT_MODE_CONTINUOUS);
        DDSOutputEnable(0, 1);
        printf("DDS0 enabled freq=%f ampl=%d\n", GetDDSFreq(0), GetDDSAmplitudeMv(0));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    unsigned mem = GetMemoryLength();
    capture("A", mem / 2 > 8 ? 8 : mem / 2, 3, 2, nch);
    if (!quick) {
        SetOscChannelRangemV(0, -1000, 1000);
        SetAcDc(1, 1);
        capture("B", 4, 1, 1, nch);
        SetAcDc(1, 0);
        if (ns > 0)
            SetOscSampleRate(rates[ns - 1]);
        capture("C", 16, 3, 2, nch);
    }
    if (IsSupportDDSDevice())
        DDSOutputEnable(0, 0);
    printf("FinishDll=%d\n", FinishDll());
    return 0;
}
