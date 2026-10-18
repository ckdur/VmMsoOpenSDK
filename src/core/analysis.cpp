#include "analysis.h"

#include <cmath>

namespace vmopen {

static const int kMinFreqAdc = 5;
static const double kNormalSensitivity = 0.8;
static const size_t kMaxEdges = 1000000;

WaveStats ComputeWaveStats(const double* b, size_t n)
{
    WaveStats s{0, 0, 0, 0};
    if (n == 0) {
        s.mean = NAN;
        s.low_mean = s.high_mean = -NAN;
        return s;
    }
    double m = b[0];
    for (size_t i = 1; i < n; i++)
        m = m + b[i];
    m = m / (double)n;
    double lo = 0, hi = 0;
    long nl = 0, nh = 0;
    for (size_t i = 0; i < n; i++) {
        if (b[i] < m) {
            lo += b[i];
            nl++;
        } else if (m < b[i]) {
            hi += b[i];
            nh++;
        }
    }
    s.mean = m;
    s.low_mean = lo / (double)nl;
    s.high_mean = hi / (double)nh;
    double v = 0;
    size_t i = 0;
    for (; i + 1 < n; i += 2)
        v = (b[i + 1] - m) * (b[i + 1] - m) + (b[i] - m) * (b[i] - m) + v;
    if (i < n)
        v = v + (b[i] - m) * (b[i] - m);
    v = v / (double)n;
    s.sd = std::sqrt(v);
    return s;
}

void WaveEdges::Update(const double* b, size_t n, double min_amp, double sens, double low_mean, double high_mean,
                       double M, double sd)
{
    double h = (high_mean - M) * sens;
    double h2 = (M - low_mean) * sens;
    if (h <= h2)
        h2 = h;
    h = h2;
    rise_.clear();
    fall_.clear();
    max_.clear();
    min_.clear();
    first_ = 0;
    avg_rise_ = avg_fall_ = -1;
    if (sd < min_amp) {
        AdjustRiseFallIndex();
        return;
    }
    enum { LOW, HIGH, DONE } st = DONE;
    size_t i = 0;
    while (i < n) {
        double v = b[i++];
        if (M <= v) {
            st = HIGH;
            break;
        }
        if (v <= M) {
            st = LOW;
            break;
        }
    }
    while (st != DONE) {
        if (st == LOW) {
            st = DONE;
            if (i >= n)
                break;
            size_t j = i;
            double v = b[j];
            bool out = false;
            while (i = j + 1, v < M) {
                if (n <= i) {
                    out = true;
                    break;
                }
                j = i;
                v = b[i];
            }
            if (out || i >= n)
                break;
            bool back = false;
            for (;;) {
                double x = b[i];
                if (!(x < h + M))
                    break;
                i++;
                if (x < M) {
                    back = true;
                    break;
                }
                if (n <= i) {
                    out = true;
                    break;
                }
            }
            if (out)
                break;
            if (back) {
                st = LOW;
                continue;
            }
            rise_.push_back((long)j);
            if (first_ == 0)
                first_ = 1;
            i++;
            if (rise_.size() < kMaxEdges)
                st = HIGH;
        } else {
            st = DONE;
            bool restart = true;
            while (restart) {
                restart = false;
                size_t j = i;
                if (!(i < n))
                    break;
                bool out = false;
                while (i = j + 1, M < b[j]) {
                    j = i;
                    if (n <= i) {
                        out = true;
                        break;
                    }
                }
                if (out || n <= i)
                    break;
                for (;;) {
                    double x = b[i];
                    if (x <= M - h) {
                        fall_.push_back((long)j);
                        if (first_ == 0)
                            first_ = 2;
                        if (fall_.size() > kMaxEdges - 1)
                            break;
                        i++;
                        st = LOW;
                        break;
                    }
                    i++;
                    if (M <= x) {
                        restart = true;
                        break;
                    }
                    if (n <= i)
                        break;
                }
            }
        }
    }

    AdjustRiseFallIndex();
    long nr = (long)rise_.size(), nf = (long)fall_.size();
    long off = first_ != 1;
    long cnt = nf - off;
    if (nr < cnt)
        cnt = nr;
    for (long k = 0; k < cnt; k++) {
        long a = rise_[k], e = fall_[k + off];
        long best = a;
        double mv = b[a];
        for (long p = a + 1; p < e; p++)
            if (mv < b[p]) {
                best = p;
                mv = b[p];
            }
        max_.push_back(best);
    }
    off = first_ != 2;
    cnt = nr - off;
    if (nf < cnt)
        cnt = nf;
    for (long k = 0; k < cnt; k++) {
        long a = fall_[k], e = rise_[k + off];
        long best = a;
        double mv = b[a];
        for (long p = a + 1; p < e; p++)
            if (b[p] < mv) {
                best = p;
                mv = b[p];
            }
        min_.push_back(best);
    }
}

void WaveEdges::AdjustRiseFallIndex()
{
    long nr = (long)rise_.size(), nf = (long)fall_.size();
    avg_rise_ = avg_fall_ = -1.0;
    if (nf > 3 && nr > 3) {
        double d = 0;
        for (long k = 0; k < nr - 1; k++)
            d = (d + (double)rise_[k + 1]) - (double)rise_[k];
        avg_rise_ = d / (double)(nr - 1);
        d = 0;
        for (long k = 0; k < nf - 1; k++)
            d = (d + (double)fall_[k + 1]) - (double)fall_[k];
        avg_fall_ = d / (double)(nf - 1);
    }
}

double WaveEdges::AveLength() const
{
    if (rise_.size() > 5 && fall_.size() > 5)
        return (avg_rise_ + avg_fall_) * 0.5;
    return 0.0;
}

MeasValue WaveEdges::Freq(unsigned rate) const
{
    MeasValue f;
    double l = AveLength();
    if (l != 0.0) {
        f.ok = true;
        f.v = (double)rate / l;
    }
    return f;
}

bool WaveEdges::RisePhase(double* phase) const
{
    *phase = 0;
    if (AveLength() == 0.0)
        return false;
    long nr = (long)rise_.size();
    if (avg_rise_ == 0.0 || nr <= 2)
        return false;
    double d = std::fmod((double)(rise_[1] * 360) / avg_rise_, 360.0);
    if (nr > 3) {
        if (d < 180.0) {
            for (long k = 2; k < nr - 1; k++)
                d = d + std::fmod((double)(rise_[k] * 360) / avg_rise_, 360.0);
        } else {
            for (long k = 2; k < nr - 1; k++) {
                double x = std::fmod((double)(rise_[k] * 360) / avg_rise_, 360.0);
                if (x < 20.0)
                    x += 360.0;
                d = d + x;
            }
        }
    }
    d = std::fmod(d / (double)(nr - 2), 360.0);
    *phase = 360.0 - d;
    return true;
}

void WaveEdges::PositiveDuty(unsigned rate, double freq, MeasValue* duty, MeasValue* width) const
{
    *duty = MeasValue();
    *width = MeasValue();
    long off = first_ != 1;
    long cnt = (long)fall_.size() - off;
    if ((long)rise_.size() < cnt)
        cnt = (long)rise_.size();
    if (cnt <= 5)
        return;
    double s = 0;
    for (long k = 1; k != cnt - 1; k++)
        s = (s + (double)fall_[k + off]) - (double)rise_[k];
    s = s / (double)(cnt - 2);
    duty->ok = true;
    duty->v = (s * 100.0) / ((double)rate / freq);
    width->ok = true;
    width->v = (s * 1000000000.0) / (double)rate;
}

void WaveEdges::NegativeDuty(unsigned rate, double freq, MeasValue* duty, MeasValue* width) const
{
    *duty = MeasValue();
    *width = MeasValue();
    long off = first_ != 2;
    long cnt = (long)rise_.size() - off;
    if ((long)fall_.size() < cnt)
        cnt = (long)fall_.size();
    if (cnt <= 5)
        return;
    double s = 0;
    for (long k = 1; k != cnt - 1; k++)
        s = (s + (double)rise_[k + off]) - (double)fall_[k];
    s = s / (double)(cnt - 2);
    duty->ok = true;
    duty->v = (s * 100.0) / ((double)rate / freq);
    width->ok = true;
    width->v = (s * 1000000000.0) / (double)rate;
}

long WaveEdges::RiseInRange(long start, long lo, long hi, long* pos) const
{
    long nr = (long)rise_.size();
    if (nr <= start)
        return -1;
    long idx = -1;
    *pos = 0;
    for (long i = start; i < nr; i++) {
        long r = rise_[i];
        if (lo <= r && r < hi) {
            *pos = r;
            idx = i;
        }
    }
    return idx;
}

// this = second waveform, other = first waveform (vendor argument order)
bool WaveEdges::PhaseDif(const WaveEdges& other, unsigned rate, double max_dev, double* out) const
{
    MeasValue f2 = Freq(rate), f1 = other.Freq(rate);
    if (!f2.is_ok() || !f1.is_ok() || max_dev < std::fabs(f2.v - f1.v))
        return false;
    long off = first_ != 1;
    long cnt = (long)fall_.size() - off;
    if ((long)rise_.size() < cnt)
        cnt = (long)rise_.size();
    if (cnt < 2)
        return false;
    std::vector<long> d;
    long start = 0;
    for (long k = 0; k < cnt - 1;) {
        long r = rise_[k];
        k++;
        long pos;
        long idx = other.RiseInRange(start, r, rise_[k], &pos);
        if (idx == -1)
            continue;
        d.push_back(pos - r);
        start = idx;
    }
    if (d.empty())
        return false;
    double sum = 0;
    for (long v : d)
        sum += (double)v;
    double mean = sum / (double)d.size();
    double s = 0;
    int c = 0;
    for (long v : d)
        if (std::fabs(mean - (double)v) < mean * 0.8) {
            s += (double)v;
            c++;
        }
    if (c == 0)
        return false;
    *out = ((s / (double)c) * 360.0) / ((double)rate / ((f2.v + f1.v) * 0.5));
    return true;
}

bool FreqMeasure::Cal(const double* buf, unsigned n, double resolution, unsigned rate)
{
    freq = MeasValue();
    phase_ok = false;
    WaveStats st = ComputeWaveStats(buf, n);
    WaveEdges w;
    w.Update(buf, n, (double)kMinFreqAdc * resolution, kNormalSensitivity, st.low_mean, st.high_mean, st.mean, st.sd);
    freq = w.Freq(rate);
    phase_ok = w.RisePhase(&phase);
    if (!freq.is_ok()) {
        pos_duty = pos_width = neg_duty = neg_width = MeasValue();
    } else {
        w.PositiveDuty(rate, freq.v, &pos_duty, &pos_width);
        w.NegativeDuty(rate, freq.v, &neg_duty, &neg_width);
    }
    return freq.is_ok();
}

int PhaseDifMeasure::Cal(const double* b1, double res1, const double* b2, double res2, unsigned n, unsigned rate,
                         double dev)
{
    freq1 = freq2 = MeasValue();
    phase_ok = false;
    phase_dif = 0;
    WaveEdges w1, w2;
    WaveStats s1 = ComputeWaveStats(b1, n);
    w1.Update(b1, n, (double)kMinFreqAdc * res1, kNormalSensitivity, s1.low_mean, s1.high_mean, s1.mean, s1.sd);
    freq1 = w1.Freq(rate);
    WaveStats s2 = ComputeWaveStats(b2, n);
    w2.Update(b2, n, (double)kMinFreqAdc * res2, kNormalSensitivity, s2.low_mean, s2.high_mean, s2.mean, s2.sd);
    freq2 = w2.Freq(rate);
    if (freq1.is_ok() && freq2.is_ok())
        phase_ok = w2.PhaseDif(w1, rate, dev, &phase_dif);
    if (freq1.is_ok() && freq2.is_ok())
        return phase_ok ? 1 : 2;
    if (freq1.is_ok())
        return 3;
    return freq2.is_ok() ? 4 : 0;
}

} // namespace vmopen
