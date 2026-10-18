// FIR filter design by the window method (fir1.h) and frequency sampling (fir2.h).
#include "fir1.h"
#include "fir2.h"

#include <cmath>
#include <vector>

#include "internal.h"
#include "vmdft.h"

namespace {
const double kPi = 3.141592653589793;

// piecewise linear interpolation; points outside [x0, xn-1] are left untouched
void Interp1(const double* x, const double* y, int n, const double* xi, double* yi, int ni)
{
    int k = 0;
    for (int j = 0; j + 1 < n; j++) {
        double dx = x[j + 1] - x[j];
        while (k < ni && xi[k] >= x[j] && xi[k] <= x[j + 1]) {
            yi[k] = y[j] + (xi[k] - x[j]) * ((y[j + 1] - y[j]) / dx);
            k++;
        }
    }
}
} // namespace

// Window method design of an N tap filter, fln/fhn normalised to Nyquist.
// Even N supports only low pass and band pass (type II).
char SIGNAL_DLL_CALL fir(int32_t N, BANDTYPE band, double fln, double fhn, WINDOW_STYLE window, double alpha_beta,
                         double* h)
{
    const double wc1 = fln * kPi, wc2 = fhn * kPi;
    const double c = (double)(N - 1) * 0.5;
    const bool odd = (N & 1) != 0;
    if (!odd && (band == HIGH || band == STOP))
        return 0;
    int last = odd ? N / 2 - 1 : N / 2;   // last index computed from the formula (inclusive)
    auto fill = [&](auto f) {
        for (int i = 0; i <= last; i++) {
            double n = (double)i - c;
            double v = f(n);
            h[i] = v;
            h[N - 1 - i] = v;
        }
    };
    double w0;
    switch (band) {
    case LOW:
        fill([&](double n) { return std::sin(wc1 * n) / (n * kPi); });
        if (odd)
            h[(N - 1) / 2] = wc1 / kPi;
        w0 = 0.0;
        break;
    case HIGH:
        fill([&](double n) { return (std::sin(n * kPi) - std::sin(n * wc1)) / (n * kPi); });
        h[(N - 1) / 2] = 1.0 - wc1 / kPi;
        w0 = kPi;
        break;
    case PASS:
        fill([&](double n) { return (std::sin(n * wc2) - std::sin(n * wc1)) / (n * kPi); });
        if (odd)
            h[(N - 1) / 2] = (wc2 - wc1) / kPi;
        w0 = ((fhn - fln) * 0.5 + fln) * kPi;
        break;
    case STOP:
        fill([&](double n) { return ((std::sin(n * wc1) + std::sin(n * kPi)) - std::sin(n * wc2)) / (n * kPi); });
        h[(N - 1) / 2] = ((wc1 + kPi) - wc2) / kPi;
        w0 = 0.0;
        break;
    default:
        w0 = ((fhn - fln) * 0.5 + fln) * kPi;
        break;
    }
    window_i(window, h, N, alpha_beta, 1);
    // unity gain at the centre of the pass band
    double re = 0, im = 0;
    for (int i = 0; i < N; i++) {
        re += std::cos(i * w0) * h[i];
        im -= std::sin(i * w0) * h[i];
    }
    double g = 1.0 / std::sqrt(re * re + im * im);
    for (int i = 0; i < N; i++)
        h[i] *= g;
    return 1;
}

// Frequency sampling design (Octave fir2): order n (n+1 taps), fm_n points
// f (0..1, increasing) with magnitudes m.
char SIGNAL_DLL_CALL fir2(double* h, int32_t n, double* f, double* m, int32_t fm_n, int32_t grid_n, double ramp_n,
                          WINDOW_STYLE window, double alpha_beta)
{
    std::vector<double> ff(f, f + fm_n);
    // spread coincident points (steps) over ramp_n grid points
    double d = (ramp_n / (double)grid_n) * 0.5;
    for (int i = 0; i + 1 < fm_n; i++)
        if (ff[i + 1] - ff[i] == 0.0) {
            ff[i] -= d;
            ff[i + 1] += d;
        }
    std::vector<double> gx(grid_n + 1), grid(grid_n + 1, 0.0);
    for (int i = 0; i <= grid_n; i++)
        gx[i] = (double)i / (double)grid_n;
    Interp1(ff.data(), m, fm_n, gx.data(), grid.data(), grid_n + 1);

    if ((n & 1) == 0) {
        int len = grid_n * 2;
        std::vector<double> re(len, 0.0), im(len, 0.0), xr(len), xi(len);
        for (int i = 0; i <= grid_n; i++)
            re[i] = grid[i];
        for (int i = 1; i < grid_n; i++)
            re[grid_n + i] = grid[grid_n - i];
        idft(re.data(), im.data(), len, xr.data(), xi.data(), len);
        int half = n / 2;
        for (int i = 0; i < half; i++)
            h[i] = xr[len - half + i];
        for (int i = 0; i <= half; i++)
            h[half + i] = xr[i];
    } else {
        int len = grid_n * 4;
        std::vector<double> re(len, 0.0), im(len, 0.0), xr(len), xi(len);
        for (int i = 0; i <= grid_n; i++)
            re[i] = grid[i];
        for (int i = 1; i < grid_n; i++)
            re[3 * grid_n + i] = grid[grid_n - i];
        idft(re.data(), im.data(), len, xr.data(), xi.data(), len);
        int a = (n + 1) / 2;
        for (int i = 0; i < a; i++)
            h[i] = 2.0 * xr[len - n + 2 * i];
        for (int i = 0; i < a; i++)
            h[a + i] = 2.0 * xr[1 + 2 * i];
    }
    window_i(window, h, n + 1, alpha_beta, 0);
    return 1;
}

// step response
void SIGNAL_DLL_CALL fir_resp(int32_t N, double* hn, double* jy)
{
    double s = 0;
    for (int i = 0; i < N; i++) {
        s += hn[i];
        jy[i] = s;
    }
}

// frequency response at HN points over [0, pi)
void SIGNAL_DLL_CALL fir_Hw(double* h, int32_t N, double* HRm, double* HIm, int32_t HN)
{
    dft_real(h, N, HRm, HIm, HN, 0);
}

void SIGNAL_DLL_CALL fir_Amplitude(double* x_Re, double* x_Im, double* af, uint32_t N, char uselog10)
{
    for (uint32_t i = 0; i < N; i++) {
        double a = std::sqrt(x_Re[i] * x_Re[i] + x_Im[i] * x_Im[i]);
        af[i] = uselog10 ? std::log10(a) * 20.0 : a;
    }
}

// Phase; unwrapped for left_N <= i <= right_N
void SIGNAL_DLL_CALL fir_Phase(double* x_Re, double* x_Im, double* pf, int32_t N, int32_t left_N, int32_t right_N)
{
    int i = 0;
    if (left_N > 0) {
        int e = left_N < N ? left_N : N;
        for (; i < e; i++)
            pf[i] = std::atan2(x_Im[i], x_Re[i]) + 0.0;
    }
    double off = 0.0;
    for (; i < N; i++) {
        double ph = std::atan2(x_Im[i], x_Re[i]);
        double v = off + ph;
        if (i <= right_N && i != 0) {
            double dlt = pf[i - 1] - v;
            if (dlt < -4.0) {
                off -= 2.0 * kPi;
                v = ph + off;
            } else if (dlt > 4.0) {
                off += 2.0 * kPi;
                v = ph + off;
            }
        }
        pf[i] = v;
    }
}

void SIGNAL_DLL_CALL fir_Phase_Daizu(double* x_Re, double* x_Im, double* pf, int32_t N, int32_t left_N,
                                     int32_t right_N)
{
    fir_Phase(x_Re, x_Im, pf, N, left_N, right_N);
}
