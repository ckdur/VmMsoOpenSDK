// Waveform measurements (frequency, phase, duty cycle, phase difference)
// using hysteresis edge detection around the mean value.
#pragma once

#include <cstdint>
#include <vector>

namespace vmopen {

struct MeasValue {
    bool ok = false;
    double v = 0;
    bool is_ok() const { return ok && v > 0.0; }
};

class WaveEdges {
public:
    // sens: hysteresis as fraction of the half amplitude, min_amp: minimum std deviation
    void Update(const double* buf, size_t n, double min_amp, double sens, double low_mean, double high_mean,
                double mean, double sd);
    MeasValue Freq(unsigned rate) const;
    bool RisePhase(double* phase) const;
    void PositiveDuty(unsigned rate, double freq, MeasValue* duty, MeasValue* width) const;
    void NegativeDuty(unsigned rate, double freq, MeasValue* duty, MeasValue* width) const;
    bool PhaseDif(const WaveEdges& other, unsigned rate, double max_freq_dev, double* out) const;

private:
    void AdjustRiseFallIndex();
    double AveLength() const;
    long RiseInRange(long start, long lo, long hi, long* pos) const;

    std::vector<long> rise_, fall_, max_, min_;
    int first_ = 0;   // 1 = rising edge first, 2 = falling edge first
    double avg_rise_ = -1, avg_fall_ = -1;
};

struct WaveStats {
    double mean, low_mean, high_mean, sd;
};
WaveStats ComputeWaveStats(const double* buf, size_t n);

// Global measurement state mirroring the C API (CalFreq / CalFreqAndPhaseDif).
struct FreqMeasure {
    MeasValue freq, pos_duty, pos_width, neg_duty, neg_width;
    bool phase_ok = false;
    double phase = 0;
    bool Cal(const double* buf, unsigned n, double resolution, unsigned rate);
};

struct PhaseDifMeasure {
    MeasValue freq1, freq2;
    bool phase_ok = false;
    double phase_dif = 0;
    int Cal(const double* b1, double res1, const double* b2, double res2, unsigned n, unsigned rate, double dev);
};

} // namespace vmopen
