// Spectrum analysis results (vmfftinfo.h): harmonics, SNR, THD, SINAD, ENOB, SFDR.
#include "vmfftinfo.h"

#include <cmath>

harmonicinfo::harmonicinfo() { clear(); }

void harmonicinfo::clear()
{
    m_freq = 0;
    m_value = 0;
    m_index = -1;
    m_start_index = -1;
    m_stop_index = -1;
}

fftinfo::fftinfo()
    : m_ref_type(FFT_Amplitude_Ref_V), m_snr(0), m_sinad(0), m_enob(0), m_thd_db(0), m_thd_percent(0), m_sfdr(0),
      m_fft_buffer_mean(0)
{
}

double fftinfo::CalPower(double* fft_buffer, const int64_t start, const int64_t stop)
{
    double p = 0;
    for (int64_t i = start; i <= stop; i++)
        p += fft_buffer[i] * fft_buffer[i];
    return p;
}

double fftinfo::CalHarmonic(double frequence, uint32_t sample, int32_t n)
{
    // fold the n-th harmonic into the first Nyquist zone
    double f = std::fmod(frequence * n, (double)sample);
    if (f > sample / 2.0)
        f = sample - f;
    return f;
}

size_t fftinfo::FineSecondMax(double* fft_buffer, size_t fft_n)
{
    size_t best = 1;
    for (size_t i = 1; i < fft_n; i++)
        if ((int64_t)i < m_first_harmonic.m_start_index || (int64_t)i > m_first_harmonic.m_stop_index)
            if (fft_buffer[i] > fft_buffer[best] || best == (size_t)m_first_harmonic.m_index)
                best = i;
    return best;
}

bool fftinfo::CalHarmonicFreqRange(double* fft_buffer, const size_t fft_n, const uint32_t, const int64_t idx,
                                   int64_t* start_index, int64_t* stop_index, const double mean, double yuzhi_biii)
{
    if (idx < 0 || (size_t)idx >= fft_n)
        return false;
    double lim = mean + (fft_buffer[idx] - mean) * yuzhi_biii;
    int64_t s = idx, e = idx;
    while (s > 0 && fft_buffer[s - 1] < fft_buffer[s] && fft_buffer[s - 1] > lim)
        s--;
    while ((size_t)(e + 1) < fft_n && fft_buffer[e + 1] < fft_buffer[e] && fft_buffer[e + 1] > lim)
        e++;
    *start_index = s;
    *stop_index = e;
    return true;
}

void fftinfo::CalInfos(double* fft_buffer, size_t fft_n, uint32_t sample, bool is_use_seven)
{
    double ps = std::sqrt(CalPower(fft_buffer, m_first_harmonic.m_start_index, m_first_harmonic.m_stop_index));
    double total = std::sqrt(CalPower(fft_buffer, 1, (int64_t)fft_n - 1) -
                             CalPower(fft_buffer, m_dc_harmonic.m_start_index, m_dc_harmonic.m_stop_index) *
                                 (m_dc_harmonic.m_start_index == 0 ? 0 : 1));
    harmonicinfo* hs[] = {&m_second_harmonic, &m_third_harmonic, &m_fourth_harmonic,
                          &m_fifth_harmonic,  &m_six_harmonic,   &m_seventh_harmonic};
    int nh = is_use_seven ? 6 : 4;
    double ph = 0;
    for (int i = 0; i < nh; i++)
        if (hs[i]->m_start_index >= 0)
            ph += CalPower(fft_buffer, hs[i]->m_start_index, hs[i]->m_stop_index);
    ph = std::sqrt(ph);
    double noise2 = total * total - ps * ps - ph * ph;
    double noise = std::sqrt(noise2 > 0 ? noise2 : 1e-300);
    double nd = std::sqrt(total * total - ps * ps > 0 ? total * total - ps * ps : 1e-300);
    m_snr = 20.0 * std::log10(ps / noise);
    m_sinad = 20.0 * std::log10(ps / nd);
    m_enob = (m_sinad - 1.76) / 6.02;
    m_thd_percent = ps > 0 ? ph / ps * 100.0 : 0;
    m_thd_db = 20.0 * std::log10(ph / ps);
    size_t spur = FineSecondMax(fft_buffer, fft_n);
    m_sfdr = 20.0 * std::log10(m_first_harmonic.m_value / fft_buffer[spur]);
    (void)sample;
}

void fftinfo::CalFftInfo(double frequence, uint32_t sample, double* fft_buffer, size_t fft_n, double R,
                         FFT_Amplitude_Ref_Type type, bool is_use_seven, double yuzhi_biii)
{
    m_ref_type = type;
    if (!fft_buffer || fft_n < 4 || sample == 0)
        return;
    double mean = 0;
    for (size_t i = 0; i < fft_n; i++)
        mean += fft_buffer[i];
    m_fft_buffer_mean = mean / (double)fft_n;
    double df = (double)sample / 2.0 / (double)fft_n;
    harmonicinfo* hs[] = {&m_first_harmonic, &m_second_harmonic, &m_third_harmonic,  &m_fourth_harmonic,
                          &m_fifth_harmonic, &m_six_harmonic,    &m_seventh_harmonic};
    m_dc_harmonic.clear();
    m_dc_harmonic.m_index = 0;
    m_dc_harmonic.m_value = fft_buffer[0];
    CalHarmonicFreqRange(fft_buffer, fft_n, sample, 0, &m_dc_harmonic.m_start_index, &m_dc_harmonic.m_stop_index,
                         m_fft_buffer_mean, yuzhi_biii);
    for (int h = 0; h < 7; h++) {
        harmonicinfo& hi = *hs[h];
        hi.clear();
        double f = CalHarmonic(frequence, sample, h + 1);
        int64_t idx = (int64_t)std::llround(f / df);
        if (idx < 1 || (size_t)idx >= fft_n)
            continue;
        // search the local maximum around the expected bin
        int64_t best = idx;
        for (int64_t i = idx - 2; i <= idx + 2; i++)
            if (i >= 1 && (size_t)i < fft_n && fft_buffer[i] > fft_buffer[best])
                best = i;
        hi.m_index = best;
        hi.m_freq = best * df;
        hi.m_value = fft_buffer[best];
        CalHarmonicFreqRange(fft_buffer, fft_n, sample, best, &hi.m_start_index, &hi.m_stop_index,
                             m_fft_buffer_mean, yuzhi_biii);
    }
    if (m_first_harmonic.m_index > 0)
        CalInfos(fft_buffer, fft_n, sample, is_use_seven);
    (void)R;
}

bool AdjHarmonicFreqAndRms(double* fft_buffer, const size_t fft_n, const uint32_t sample, const double first_harmonic,
                           int64_t* freq_index, double* adj_freq, double* adj_rms)
{
    if (!fft_buffer || fft_n < 3)
        return false;
    double df = (double)sample / 2.0 / (double)fft_n;
    int64_t i = (int64_t)std::llround(first_harmonic / df);
    if (i < 1 || (size_t)i + 1 >= fft_n)
        return false;
    // parabolic interpolation of the peak
    double a = fft_buffer[i - 1], b = fft_buffer[i], c = fft_buffer[i + 1];
    double den = a - 2 * b + c;
    double d = den != 0 ? 0.5 * (a - c) / den : 0;
    *freq_index = i;
    *adj_freq = (i + d) * df;
    *adj_rms = std::sqrt(a * a + b * b + c * c);
    return true;
}
