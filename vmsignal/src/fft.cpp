// FFT, DFT and Hartley transforms (vmfft.h, vmreal_fft.h, vmfht.h, vmdft.h).
// The vendor library offers several algorithms selected by 'flag'; they all
// compute the same transform, so a single implementation is used here.
#include "vmfft.h"
#include "vmcal_m.h"
#include "vmdft.h"
#include "vmfht.h"
#include "vmreal_fft.h"

#include <cmath>
#include <cstring>
#include <vector>

#include "internal.h"

namespace {

const double kPi = 3.141592653589793;

bool IsPow2(uint32_t n) { return n && !(n & (n - 1)); }

// cos/sin of -2*pi*k/n for k < n/2, cached per thread for the last sizes used
const std::vector<double>& Twiddles(uint32_t n)
{
    thread_local std::vector<std::pair<uint32_t, std::vector<double>>> cache;
    for (auto& e : cache)
        if (e.first == n)
            return e.second;
    if (cache.size() >= 4)
        cache.erase(cache.begin());
    std::vector<double> t(n);   // interleaved cos, sin
    for (uint32_t k = 0; k < n / 2; k++) {
        double a = -2.0 * kPi * (double)k / (double)n;
        t[2 * k] = std::cos(a);
        t[2 * k + 1] = std::sin(a);
    }
    cache.emplace_back(n, std::move(t));
    return cache.back().second;
}

void Radix2(double* re, double* im, uint32_t n, int sign)
{
    for (uint32_t i = 1, j = 0; i < n; i++) {
        uint32_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            std::swap(re[i], re[j]);
            std::swap(im[i], im[j]);
        }
    }
    const double* tw = Twiddles(n).data();
    const double sg = sign < 0 ? 1.0 : -1.0;   // table holds the forward twiddles
    for (uint32_t i = 0; i + 1 < n; i += 2) {
        double tr = re[i + 1], ti = im[i + 1];
        re[i + 1] = re[i] - tr;
        im[i + 1] = im[i] - ti;
        re[i] += tr;
        im[i] += ti;
    }
    for (uint32_t len = 4; len <= n; len <<= 1) {
        uint32_t half = len >> 1, step = n / len;
        for (uint32_t i = 0; i < n; i += len) {
            double* r0 = re + i;
            double* i0 = im + i;
            double* r1 = r0 + half;
            double* i1 = i0 + half;
            for (uint32_t k = 0; k < half; k++) {
                double wr = tw[2 * k * step], wi = sg * tw[2 * k * step + 1];
                double tr = r1[k] * wr - i1[k] * wi;
                double ti = r1[k] * wi + i1[k] * wr;
                r1[k] = r0[k] - tr;
                i1[k] = i0[k] - ti;
                r0[k] += tr;
                i0[k] += ti;
            }
        }
    }
}

void Direct(double* re, double* im, uint32_t n, int sign)
{
    std::vector<double> r(n), m(n);
    for (uint32_t k = 0; k < n; k++) {
        double sr = 0, si = 0;
        for (uint32_t t = 0; t < n; t++) {
            double a = sign * 2.0 * kPi * (double)(((uint64_t)k * t) % n) / (double)n;
            double c = std::cos(a), s = std::sin(a);
            sr += re[t] * c - im[t] * s;
            si += re[t] * s + im[t] * c;
        }
        r[k] = sr;
        m[k] = si;
    }
    std::memcpy(re, r.data(), n * sizeof(double));
    std::memcpy(im, m.data(), n * sizeof(double));
}

} // namespace

void vmsig_fft(double* re, double* im, uint32_t n, int sign)
{
    if (n < 2)
        return;
    if (IsPow2(n))
        Radix2(re, im, n, sign);
    else
        Direct(re, im, n, sign);
}

int32_t SIGNAL_DLL_CALL CAL_2M(int32_t N)
{
    int m = -1;
    for (; N; N >>= 1)
        m++;
    return m;
}

int32_t SIGNAL_DLL_CALL CAL_4M(int32_t N)
{
    int m = -1;
    for (; N; N >>= 2)
        m++;
    return m;
}

void SIGNAL_DLL_CALL dit2_fft(double* x_Re, double* x_Im, uint32_t N, char sign)
{
    vmsig_fft(x_Re, x_Im, N, sign == 1 ? -1 : (sign == 0 ? -1 : 1));
}

void SIGNAL_DLL_CALL dit2_ifft(double* x_Re, double* x_Im, uint32_t N)
{
    vmsig_fft(x_Re, x_Im, N, 1);
    for (uint32_t i = 0; i < N; i++) {
        x_Re[i] /= (double)N;
        x_Im[i] /= (double)N;
    }
}

void SIGNAL_DLL_CALL dit4_fft(double* x_Re, double* x_Im, uint32_t N) { vmsig_fft(x_Re, x_Im, N, -1); }
void SIGNAL_DLL_CALL sr_fft(double* x_Re, double* x_Im, uint32_t N) { vmsig_fft(x_Re, x_Im, N, -1); }
void SIGNAL_DLL_CALL fft(double* x_Re, double* x_Im, uint32_t N, char) { vmsig_fft(x_Re, x_Im, N, -1); }

void SIGNAL_DLL_CALL ifft(double* x_Re, double* x_Im, uint32_t N, char)
{
    if (N == 0)
        return;
    vmsig_fft(x_Re, x_Im, N, 1);
    for (uint32_t i = 0; i < N; i++) {
        x_Re[i] /= (double)N;
        x_Im[i] /= (double)N;
    }
}

void SIGNAL_DLL_CALL fft_signal(double* x_Re, double* x_Im, uint32_t N, uint32_t Real_N, char flag)
{
    fft(x_Re, x_Im, N, flag);
    for (uint32_t i = 0; i < N; i++) {
        x_Re[i] /= (double)Real_N;
        x_Im[i] /= (double)Real_N;
    }
}

// FFT of a real sequence x_Re[0..N-1]; x_Im is ignored on input. For a
// power of two the N point transform is computed with one N/2 point complex FFT.
void SIGNAL_DLL_CALL real_fft(double* x_Re, double* x_Im, uint32_t N, char)
{
    if (N < 4 || !IsPow2(N)) {
        std::memset(x_Im, 0, N * sizeof(double));
        vmsig_fft(x_Re, x_Im, N, -1);
        if (N >= 2) {
            x_Im[0] = 0.0;
            x_Im[N / 2] = 0.0;
        }
        return;
    }
    const uint32_t h = N / 2;
    // pack even/odd samples into the upper halves as one complex sequence
    double* zr = x_Re + h;
    double* zi = x_Im + h;
    std::vector<double> tmp(x_Re, x_Re + N);
    for (uint32_t m = 0; m < h; m++) {
        zr[m] = tmp[2 * m];
        zi[m] = tmp[2 * m + 1];
    }
    vmsig_fft(zr, zi, h, -1);
    const double* tw = Twiddles(N).data();
    // X[k] = E[k] + W^k O[k], E = (Z[k] + conj Z[h-k]) / 2, O = (Z[k] - conj Z[h-k]) / 2j
    std::vector<double> outr(h + 1), outi(h + 1);
    for (uint32_t k = 0; k <= h; k++) {
        uint32_t a = k % h, b = (h - k) % h;
        double ar = zr[a], ai = zi[a], br = zr[b], bi = -zi[b];
        double er = (ar + br) * 0.5, ei = (ai + bi) * 0.5;
        double or_ = (ai - bi) * 0.5, oi = -(ar - br) * 0.5;
        double wr, wi;
        if (k < h) {
            wr = tw[2 * k];
            wi = tw[2 * k + 1];
        } else {
            wr = -1.0;
            wi = 0.0;
        }
        outr[k] = er + (or_ * wr - oi * wi);
        outi[k] = ei + (or_ * wi + oi * wr);
    }
    for (uint32_t k = 0; k <= h; k++) {
        x_Re[k] = outr[k];
        x_Im[k] = outi[k];
    }
    for (uint32_t k = 1; k < h; k++) {
        x_Re[N - k] = outr[k];
        x_Im[N - k] = -outi[k];
    }
    x_Im[0] = 0.0;
    x_Im[h] = 0.0;
}

void SIGNAL_DLL_CALL real_fft_fht(double* x_Re, double* x_Im, uint32_t N, char flag) { real_fft(x_Re, x_Im, N, flag); }

void SIGNAL_DLL_CALL real_fft_signal(double* x_Re, double* x_Im, uint32_t N, uint32_t Real_N, char flag)
{
    real_fft(x_Re, x_Im, N, flag);
    for (uint32_t i = 0; i < N; i++) {
        x_Re[i] /= (double)Real_N;
        x_Im[i] /= (double)Real_N;
    }
}

void SIGNAL_DLL_CALL Fft_Amplitude(double* x_Re, double* x_Im, double* af, uint32_t N, unsigned char rms)
{
    for (uint32_t i = 0; i < N; i++) {
        double p = x_Re[i] * x_Re[i] + x_Im[i] * x_Im[i];
        af[i] = std::sqrt(rms ? p * 0.5 : p);
    }
}

double SIGNAL_DLL_CALL V_Conver_dBs(double v, double R, FFT_Amplitude_Ref_Type type)
{
    switch (type) {
    case FFT_Amplitude_Ref_dBV: return std::log10(v) * 20.0;
    case FFT_Amplitude_Ref_dBmV: return std::log10(v * 1000.0) * 20.0;
    case FFT_Amplitude_Ref_dBm: return std::log10((v * v * 1000.0) / R) * 10.0;
    case FFT_Amplitude_Ref_dBu: return std::log10(v / 0.775) * 20.0;
    default: return v;
    }
}

void SIGNAL_DLL_CALL Fft_Amplitude_Conver_Type(double* af, uint32_t len, double R, FFT_Amplitude_Ref_Type type)
{
    if (type == FFT_Amplitude_Ref_V)
        return;
    for (uint32_t i = 0; i < len; i++)
        af[i] = V_Conver_dBs(af[i], R, type);
}

void SIGNAL_DLL_CALL Fft_Phase(double* x_Re, double* x_Im, double* af, double* pf, uint32_t N,
                               FFT_Phase_Ref_Type ref_type)
{
    for (uint32_t i = 0; i < N; i++) {
        if (af[i] <= 0.001) {
            pf[i] = 0.0;
            continue;
        }
        double ph = std::atan2(x_Im[i], x_Re[i]);
        pf[i] = ref_type == FFT_Phase_Ref_Rad ? ph : (ph * 180.0) / kPi;
    }
}

// X[k] = sum x[i] exp(-j*i*k*w/N), w = 2*pi (flag 1) or pi (half spectrum)
void dft_real(double* x, int32_t n, double* XRe, double* XIm, int32_t N, char flag)
{
    double w = (flag == 1 ? 2.0 * kPi : kPi) / (double)N;
    for (int32_t k = 0; k < N; k++) {
        double re = 0, im = 0;
        for (int32_t i = 0; i < n; i++) {
            double a = (double)i * (double)k * w;
            re += std::cos(a) * x[i];
            im += std::sin(a) * x[i];
        }
        XRe[k] = re;
        XIm[k] = -im;
    }
}

void idft(double* XRe, double* XIm, int32_t N, double* xre, double* xim, int32_t n)
{
    std::vector<double> r(XRe, XRe + N), m(XIm, XIm + N);
    vmsig_fft(r.data(), m.data(), (uint32_t)N, 1);
    for (int32_t i = 0; i < n && i < N; i++) {
        xre[i] = r[i] / (double)N;
        xim[i] = m[i] / (double)N;
    }
}

// Discrete Hartley transform; sign 1 forward, otherwise inverse (scaled by 1/n)
void SIGNAL_DLL_CALL fht(double* x, uint32_t n, char, char sign)
{
    std::vector<double> im(n, 0.0);
    vmsig_fft(x, im.data(), n, -1);
    for (uint32_t i = 0; i < n; i++) {
        x[i] = x[i] - im[i];
        if (sign != 1)
            x[i] /= (double)n;
    }
}

void SIGNAL_DLL_CALL dit2_fht(double* x, uint32_t n) { fht(x, n, 0, 1); }
void SIGNAL_DLL_CALL dit4_fht(double* x, uint32_t n) { fht(x, n, 1, 1); }
void SIGNAL_DLL_CALL sr_fht(double* x, uint32_t n) { fht(x, n, 2, 1); }

void SIGNAL_DLL_CALL fht_dft(double* XRe, double* XIm, uint32_t N, char flag) { real_fft(XRe, XIm, N, flag); }

void vmsig_convolve_inplace(const double* h, int32_t hn, double* x, int32_t num, int32_t fft_n)
{
    if (num <= 0 || hn <= 0)
        return;
    uint32_t n = 1;
    while ((int64_t)n < (int64_t)fft_n || (int32_t)n < hn * 2)
        n <<= 1;
    int32_t L = (int32_t)n + 1 - hn;   // new samples per block
    std::vector<double> hr(n, 0.0), hi(n, 0.0);
    for (int32_t i = 0; i < hn; i++)
        hr[i] = h[i];
    vmsig_fft(hr.data(), hi.data(), n, -1);
    std::vector<double> out(num, 0.0), br(n), bi(n);
    for (int32_t start = 0; start < num; start += L) {
        int32_t cnt = num - start < L ? num - start : L;
        std::fill(br.begin(), br.end(), 0.0);
        std::fill(bi.begin(), bi.end(), 0.0);
        for (int32_t i = 0; i < cnt; i++)
            br[i] = x[start + i];
        vmsig_fft(br.data(), bi.data(), n, -1);
        for (uint32_t k = 0; k < n; k++) {
            double r = br[k] * hr[k] - bi[k] * hi[k];
            double m = br[k] * hi[k] + bi[k] * hr[k];
            br[k] = r;
            bi[k] = m;
        }
        vmsig_fft(br.data(), bi.data(), n, 1);
        for (uint32_t i = 0; i < n && start + (int32_t)i < num; i++)
            out[start + i] += br[i] / (double)n;
    }
    std::memcpy(x, out.data(), num * sizeof(double));
}
