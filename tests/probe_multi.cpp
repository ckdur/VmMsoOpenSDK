// Exercises the multi device API (VmMultMsoLib.h): DDS waveforms, IO, DAC,
// normal trigger with abort, and stream mode. Runs with the vendor
// libvmmultmso or the open implementation so outputs and USB traces can be diffed.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
#include "VmMultMsoLib.h"

static std::atomic<unsigned> dev{0};
static std::atomic<int> added{0}, ready{0}, io_ready{0};
static std::atomic<unsigned> io_state{0};
static std::atomic<unsigned long long> stream_total{0}, stream_need{0};
static std::atomic<unsigned> stream_blocks{0}, stream_failed{0}, raw_blocks{0};
static double stream_first[2] = {1e9, 1e9};

static void CALLBACK on_add(void*, unsigned int id)
{
    dev = id;
    added = 1;
}
static void CALLBACK on_rmv(void*, unsigned int) { added = 0; }
static void CALLBACK on_data(void*, unsigned int) { ready++; }
static void CALLBACK on_io(void*, unsigned int, unsigned int st)
{
    io_state = st;
    io_ready++;
}
static void CALLBACK on_stream(void*, unsigned int, unsigned char ch, double* b, unsigned int n, unsigned int failed,
                               unsigned int, unsigned long long need, unsigned long long total, unsigned long long)
{
    if (ch < 2 && stream_first[ch] > 1e8 && n)
        stream_first[ch] = b[0];
    if (ch == 0)
        stream_blocks++;
    stream_failed = failed;
    stream_need = need;
    stream_total = total;
}
static void CALLBACK on_raw(void*, unsigned int, unsigned char*, unsigned int, unsigned int, unsigned int,
                            unsigned long long, unsigned long long, unsigned long long)
{
    raw_blocks++;
}

static bool wait_for(std::atomic<int>& v, int ms)
{
    for (int i = 0; i < ms / 10 && !v; i++)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return v != 0;
}
static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

int main()
{
    printf("InitDll=%d\n", InitDll(0, 0));
    SetDevNoticeCallBack(nullptr, on_add, on_rmv);
    SetDataReadyCallBack(nullptr, on_data);
    SetIOReadStateCallBack(nullptr, on_io);
    SetStreamDataReadyCallBack(nullptr, on_stream);
    SetStreamRawDataReadyCallBack(nullptr, on_raw);
    printf("ScanDevice=%d\n", ScanDevice());
    if (!wait_for(added, 8000)) {
        printf("no device\n");
        FinishDll();
        return 1;
    }
    unsigned id = dev;
    char des[64] = {0};
    GetDeviceDesString(id, des, sizeof(des) - 1);
    printf("dev '%s' adcbit=%d mode=%d\n", des, GetOscAdcBit(id), GetOscCaptureMode(id));

    // ---- DDS: every waveform, uploads compared via the USB trace
    int styles[16];
    int ns = GetDDSSupportBoxingStyle(id, styles);
    SetDDSFreq(id, 0, 10000);
    SetDDSAmplitudeMv(id, 0, 1500);
    SetDDSBiasMv(id, 0, 200);
    SetDDSDutyCycle(id, 0, 30);
    DDSOutputEnable(id, 0, 1);
    for (int i = 0; i < ns; i++) {
        SetDDSBoxingStyle(id, 0, styles[i]);
        sleep_ms(150);
        printf("DDS style %x -> box=%x ampl=%d bias=%d duty=%.1f\n", styles[i], GetDDSBoxingStyle(id, 0),
               GetDDSAmplitudeMv(id, 0), GetDDSBiasMv(id, 0), GetDDSDutyCycle(id, 0));
    }
    SetDDSBoxingStyle(id, 0, BX_SINE);
    SetDDSPhase(id, 0, 90);
    SetDDSOutMode(id, 0, DDS_OUT_MODE_SWEEP);
    SetDDSSweepStartFreq(id, 0, 1000);
    SetDDSSweepStopFreq(id, 0, 20000);
    SetDDSSweepTime(id, 0, 50000000);
    sleep_ms(100);
    SetDDSOutMode(id, 0, DDS_OUT_MODE_BURST);
    SetDDSLoopsNum(id, 0, 3);
    SetDDSBurstPeriodNs(id, 0, 2000000);
    SetDDSBurstDelayNs(id, 0, 1000);
    SetDDSTriggerSource(id, 0, DDS_TRIGGER_SOURCE_MANUAL);
    DDSManualTrigger(id, 0);
    sleep_ms(100);
    SetDDSOutMode(id, 0, DDS_OUT_MODE_CONTINUOUS);
    SetDDSTriggerSource(id, 0, DDS_TRIGGER_SOURCE_INTERNAL);
    printf("DDS sweep=%.0f..%.0f t=%llu loops=%llu period=%llu delay=%llu src=%u\n", GetDDSSweepStartFreq(id, 0),
           GetDDSSweepStopFreq(id, 0), GetDDSSweepTime(id, 0), GetDDSLoopsNum(id, 0), GetDDSBurstPeriodNs(id, 0),
           GetDDSBurstDelayNs(id, 0), GetDDSTriggerSource(id, 0));

    // ---- IO / DAC
    for (int i = 0; i < 4; i++)
        SetIOInOut(id, i, 0);
    for (int i = 4; i < 8; i++) {
        SetIOInOut(id, i, 1);
        SetIOOutState(id, i, i & 1);
    }
    SetIOPulseData(id, 6, 1000, 25);
    SetIOPulseSyn(id);
    DACEnable(id, 0, 1);
    SetDACmV(id, 0, 1234);
    io_ready = 0;
    SendReadIOInStateCmd(id);
    if (wait_for(io_ready, 2000))
        printf("IO state=%08x ready=%d get=%08x\n", (unsigned)io_state, IsIOReadStateReady(id), GetIOInState(id));
    else
        printf("IO read timeout\n");
    printf("IO pulse %.1f %.1f inout4=%d dac=%d en=%d\n", GetIOPulseFreq(id, 6), GetIOPulseDuty(id, 6),
           GetIOInOut(id, 4), GetDACmV(id, 0), IsDACEnable(id, 0));
    DACEnable(id, 0, 0);

    // ---- normal trigger that never fires, then abort
    SetOscSampleRate(id, 1000000);
    SetTriggerMode(id, TRIGGER_MODE_LIANXU);
    SetTriggerSource(id, TRIGGER_SOURCE_CH1);
    SetTriggerStyle(id, TRIGGER_STYLE_RISE_EDGE);
    SetTriggerLevelmV(id, 4000, 100);
    SetPreTriggerPercent(id, 30);
    ready = 0;
    int r = Capture(id, 4, 1, 1);
    sleep_ms(1000);
    printf("[N] Capture -> %d ready=%d; abort=%d\n", r, (int)ready, AbortCapture(id));
    sleep_ms(300);
    SetTriggerMode(id, TRIGGER_MODE_AUTO);
    SetTriggerLevelmV(id, 0, 100);
    ready = 0;
    r = Capture(id, 4, 3, 1);
    printf("[N2] Capture -> %d ready=%d tp=%.2f\n", r, wait_for(ready, 3000), ReadVoltageDatasTriggerPoint(id));

    // ---- stream mode
    printf("SetOscCaptureMode=%d mode=%d\n", SetOscCaptureMode(id, 1), GetOscCaptureMode(id));
    unsigned rates[32];
    int nr = GetStreamSupportSampleRates(id, rates, 32);
    printf("stream rates(%d/%d):", nr, GetStreamSupportSampleRateNum(id));
    for (int i = 0; i < nr; i++)
        printf(" %u", rates[i]);
    printf("\nstream range=[%d,%d] acdc=%d zoom=%.6f bias=%.6f\n", GetStreamRangeMinmV(id), GetStreamRangeMaxmV(id),
           IsStreamSupportAcDc(id, 0), ReadStreamADCToVoltageZoom(id, 0), ReadStreamADCToVoltageBias(id, 0));
    SetStreamChannelRangemV(id, 0, -5000, 5000);
    SetStreamChannelRangemV(id, 1, -5000, 5000);
    printf("StreamCapture(bad rate)=%d\n", StreamCapture(id, 10, 1024, 3, 123));
    r = StreamCapture(id, 10, 2048, 3, 1000000);
    printf("StreamCapture=%d\n", r);
    for (int i = 0; i < 100 && (stream_need == 0 || stream_total < stream_need); i++)
        sleep_ms(50);
    StreamStopCapture(id);
    sleep_ms(300);
    printf("stream blocks=%u raw=%u total=%llu need=%llu failed=%u first=%.3f,%.3f\n", (unsigned)stream_blocks,
           (unsigned)raw_blocks, (unsigned long long)stream_total, (unsigned long long)stream_need,
           (unsigned)stream_failed, stream_first[0], stream_first[1]);
    printf("SetOscCaptureMode=%d mode=%d\n", SetOscCaptureMode(id, 0), GetOscCaptureMode(id));
    ready = 0;
    r = Capture(id, 4, 3, 1);
    printf("[P] Capture -> %d ready=%d\n", r, wait_for(ready, 3000));
    DDSOutputEnable(id, 0, 0);
    printf("FinishDll=%d\n", FinishDll());
    return 0;
}
