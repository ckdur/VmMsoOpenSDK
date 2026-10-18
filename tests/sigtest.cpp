// Differential test of libvmsignal: prints the results of the functions used
// by VimuLink for a fixed set of inputs. Build against the vendor and the open
// library and compare the outputs with tests/sigcompare.py.
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "vmfilter_def.h"
#include "vmcplx_vector.h"
#include "butter.h"
#include "buttord.h"
#include "cheb1ord.h"
#include "cheb2ord.h"
#include "cheby1.h"
#include "cheby2.h"
#include "ellip.h"
#include "ellipord.h"
#include "fir1.h"
#include "fir2.h"
#include "vmfft.h"
#include "vmfilters.h"
#include "vmfreqz.h"
#include "vmreal_fft.h"
#include "vmresp.h"
#include "vmsuanfa.h"
#include "vmwindow.h"

using cplx = std::complex<double>;

static void pr(const char* tag, const double* v, size_t n)
{
    printf("%s:", tag);
    for (size_t i = 0; i < n; i++)
        printf(" %.12g", v[i]);
    printf("\n");
}
static void pr(const char* tag, const std::vector<double>& v) { pr(tag, v.data(), v.size()); }

static void pr_roots(const char* tag, std::vector<cplx> r)
{
    std::sort(r.begin(), r.end(), [](const cplx& a, const cplx& b) {
        double ar = std::round(a.real() * 1e6), br = std::round(b.real() * 1e6);
        if (ar != br)
            return ar < br;
        return a.imag() < b.imag();
    });
    printf("%s:", tag);
    for (auto& c : r)
        printf(" %.10g %.10g", c.real(), c.imag());
    printf("\n");
}

// same as the GUI's zp2tfz
static void zp2tf(const cplx_vector& z, const cplx_vector& p, double g, std::vector<double>& b,
                  std::vector<double>& a)
{
    auto poly = [](const cplx_vector& r) {
        std::vector<cplx> c(1, 1.0);
        for (auto& x : r) {
            c.push_back(0.0);
            for (size_t i = c.size() - 1; i > 0; i--)
                c[i] -= x * c[i - 1];
        }
        return c;
    };
    auto cb = poly(z), ca = poly(p);
    b.clear();
    a.clear();
    for (auto& c : cb)
        b.push_back(c.real() * g);
    for (auto& c : ca)
        a.push_back(c.real());
}

int main()
{
    char tag[128];
    // ---------------------------------------------------------------- windows
    for (int w = 0; w <= 17; w++) {
        double mn = -1, mx = -1;
        getpararange((WINDOW_STYLE)w, &mn, &mx);
        double par = getdefaultpara((WINDOW_STYLE)w);
        printf("winpara%d: %.12g %.12g %.12g\n", w, par, mn, mx);
        for (int N : {15, 16, 33}) {
            std::vector<double> h(N, -9.0);
            char r = window((WINDOW_STYLE)w, N, h.data(), par, 0);
            snprintf(tag, sizeof(tag), "win%d_%d_r%d", w, N, r);
            pr(tag, h);
            std::vector<double> x(N, 1.5);
            double cg = window_i((WINDOW_STYLE)w, x.data(), N, par, 1);
            snprintf(tag, sizeof(tag), "wini%d_%d", w, N);
            printf("%s: %.12g\n", tag, cg);
        }
    }
    // -------------------------------------------------------------------- fft
    {
        const int N = 256, RN = 200;
        std::vector<double> re(N, 0.0), im(N, 0.0);
        for (int i = 0; i < RN; i++)
            re[i] = 1.3 * std::sin(2 * M_PI * 13.3 * i / RN) + 0.2 * std::cos(2 * M_PI * 41 * i / RN) + 0.1;
        double g = window_i(HANNING, re.data(), RN, 0, 0);
        printf("fftgain: %.12g\n", g);
        real_fft_signal(re.data(), im.data(), N, RN, 8);
        pr("fft_re", re);
        pr("fft_im", im);
        for (int rms = 0; rms < 2; rms++) {
            std::vector<double> mag(N);
            Fft_Amplitude(re.data(), im.data(), mag.data(), N, rms);
            snprintf(tag, sizeof(tag), "fftmag%d", rms);
            pr(tag, mag.data(), N / 2);
            for (int t : {1, 2, 4, 8, 16}) {
                std::vector<double> m2(mag.begin(), mag.begin() + N / 2);
                Fft_Amplitude_Conver_Type(m2.data(), N / 2, 50, (FFT_Amplitude_Ref_Type)t);
                snprintf(tag, sizeof(tag), "fftconv%d_%d", rms, t);
                pr(tag, m2);
            }
        }
    }
    // -------------------------------------------------------------------- fir
    std::vector<std::vector<double>> firs;
    for (int N : {31, 32}) {
        for (int band = 0; band < 4; band++) {
            for (int w : {RECTANGLE, HAMMING, KAISER, DOLPH_CHEBYSHEV, BLACKMAN}) {
                std::vector<double> h(N, 0.0);
                char r = fir(N, (BANDTYPE)band, 0.2, 0.55, (WINDOW_STYLE)w, getdefaultpara((WINDOW_STYLE)w), h.data());
                snprintf(tag, sizeof(tag), "fir%d_%d_%d_r%d", N, band, w, r);
                if (!r)
                    continue;
                pr(tag, h);
                firs.push_back(h);
                const int L = 128;
                std::vector<double> hr(2 * L), hi(2 * L), am(L), db(L), ph(L), jy(N);
                fir_Hw(h.data(), N, hr.data(), hi.data(), L);
                fir_Amplitude(hr.data(), hi.data(), am.data(), L, 0);
                fir_Amplitude(hr.data(), hi.data(), db.data(), L, 1);
                snprintf(tag, sizeof(tag), "firam%d_%d_%d", N, band, w);
                pr(tag, am);
                snprintf(tag, sizeof(tag), "firdb%d_%d_%d", N, band, w);
                pr(tag, db);
                fir_Hw(h.data(), N, hr.data(), hi.data(), 2 * L);
                fir_Phase(hr.data(), hi.data(), ph.data(), L, 0, L);
                snprintf(tag, sizeof(tag), "firph%d_%d_%d", N, band, w);
                pr(tag, ph);
                fir_resp(N, h.data(), jy.data());
                snprintf(tag, sizeof(tag), "firjy%d_%d_%d", N, band, w);
                pr(tag, jy);
            }
        }
    }
    for (int n : {30, 31}) {
        double f[] = {0, 0.1, 0.3, 0.5, 0.8, 1.0};
        double m[] = {1, 1, 0.5, 0.0, 0.2, 0.2};
        for (int w : {HAMMING, RECTANGLE, KAISER}) {
            std::vector<double> h(n + 1, 0.0);
            fir2(h.data(), n, f, m, 6, 512, 512 / 120.0, (WINDOW_STYLE)w, getdefaultpara((WINDOW_STYLE)w));
            snprintf(tag, sizeof(tag), "fir2_%d_%d", n, w);
            pr(tag, h);
            firs.push_back(h);
        }
    }
    // ------------------------------------------------------------------ roots
    int idx = 0;
    for (auto& h : firs) {
        int n = (int)h.size() - 1;
        while (n > 0 && std::fabs(h[n]) < 1e-9)
            n--;
        std::vector<double> re(n), im(n);
        int r = qrrt(n, h.data(), re.data(), im.data(), 0.00001, 100);
        std::vector<cplx> roots;
        for (int i = 0; i < n; i++)
            roots.push_back(cplx(re[i], im[i]));
        snprintf(tag, sizeof(tag), "roots%d_r%d", idx++, r);
        if (r > 0)
            pr_roots(tag, roots);
        else
            printf("%s:\n", tag);
    }
    // -------------------------------------------------------------------- iir
    struct Spec {
        int band;
        double wp1, wp2, ws1, ws2;
    } specs[] = {{LOW, 0.2, 0, 0.3, 0}, {HIGH, 0.4, 0, 0.3, 0}, {PASS, 0.3, 0.5, 0.2, 0.6}, {STOP, 0.2, 0.6, 0.3, 0.5}};
    double ap = 1.0, as = 40.0;
    for (auto& s : specs) {
        for (int style = 0; style < 4; style++) {
            double wn1 = 0, wn2 = 0;
            int N = 0;
            BANDTYPE band = (BANDTYPE)s.band;
            if (style == 0)
                N = buttord(band, s.wp1, s.wp2, s.ws1, s.ws2, ap, as, 'z', wn1, wn2);
            else if (style == 1)
                N = cheb1ord(band, s.wp1, s.wp2, s.ws1, s.ws2, ap, as, 'z', wn1, wn2);
            else if (style == 2)
                N = cheb2ord(band, s.wp1, s.wp2, s.ws1, s.ws2, ap, as, 'z', wn1, wn2);
            else
                N = ellipord(band, s.wp1, s.wp2, s.ws1, s.ws2, ap, as, 'z', wn1, wn2);
            printf("ord%d_%d: %d %.12g %.12g\n", s.band, style, N, wn1, wn2);
            if (N <= 0)
                continue;
            cplx_vector z, p;
            double g = 0;
            if (style == 0)
                butter(N, wn1, wn2, band, 'z', z, p, g);
            else if (style == 1)
                cheby1(N, wn1, wn2, ap, band, 'z', z, p, g);
            else if (style == 2)
                cheby2(N, wn1, wn2, as, band, 'z', z, p, g);
            else
                ellip(N, wn1, wn2, ap, as, band, 'z', z, p, g);
            printf("design%d_%d: nz=%zu np=%zu g=%.10g\n", s.band, style, z.size(), p.size(), g);
            snprintf(tag, sizeof(tag), "z%d_%d", s.band, style);
            pr_roots(tag, std::vector<cplx>(z.begin(), z.end()));
            snprintf(tag, sizeof(tag), "p%d_%d", s.band, style);
            pr_roots(tag, std::vector<cplx>(p.begin(), p.end()));
            std::vector<double> b, a;
            zp2tf(z, p, g, b, a);
            snprintf(tag, sizeof(tag), "b%d_%d", s.band, style);
            pr(tag, b);
            snprintf(tag, sizeof(tag), "a%d_%d", s.band, style);
            pr(tag, a);
            for (int sign = 0; sign < 3; sign++) {
                auto r = freqz(b, a, 100, sign);
                snprintf(tag, sizeof(tag), "freqz%d_%d_%d", s.band, style, sign);
                pr(tag, r);
            }
            for (char mj = 0; mj < 2; mj++) {
                auto y = resp(b, a, 60, mj);
                snprintf(tag, sizeof(tag), "resp%d_%d_%d", s.band, style, mj);
                pr(tag, y);
            }
            // overlap-add IIR filtering
            std::vector<double> x(3000);
            for (size_t i = 0; i < x.size(); i++)
                x[i] = std::sin(i * 0.05) + 0.3 * std::sin(i * 1.1) + ((i * 7919) % 13) * 0.01;
            filters_overlap_add_fht(b.data(), (int)b.size(), a.data(), (int)a.size(), x.data(), (int)x.size(), 4096);
            snprintf(tag, sizeof(tag), "iirfilt%d_%d", s.band, style);
            pr(tag, x.data() + 2900, 100);
        }
    }
    // ------------------------------------------------------------ FIR filtering
    {
        std::vector<double> h = firs[1];
        for (int num : {1000, 10000}) {
            std::vector<double> x(num);
            for (int i = 0; i < num; i++)
                x[i] = std::sin(i * 0.03) + 0.5 * std::sin(i * 2.0) + (i % 17) * 0.02;
            filters_overlap_add_fht(h.data(), (int)h.size(), x.data(), num, 4096);
            snprintf(tag, sizeof(tag), "firfilt%d", num);
            pr(tag, x.data() + num - 200, 200);
            pr("firfilt_head", x.data(), 50);
        }
    }
    return 0;
}
