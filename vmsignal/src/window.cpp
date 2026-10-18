// Window functions (vmwindow.h). Windows are symmetric: the first half is
// computed and mirrored, like the vendor library.
#include "vmwindow.h"

#include <cmath>
#include <cstdlib>
#include <vector>

#include "internal.h"

namespace {

const double kPi = 3.141592653589793;

// modified Bessel function of the first kind, order 0 (rational approximations)
const double i0p[14] = {0.9999999999999997,   0.2466405579426905,    0.01478980363444585,  0.000382699355994036,
                        5.395676869878828e-06, 4.700912200921704e-08,  2.733894920915608e-10, 1.115830108455192e-12,
                        3.301093025084127e-15, 7.209167098020555e-18,  1.166898488777214e-20, 1.378948246502109e-23,
                        1.124884061857506e-26, 5.498556929587117e-30};
const double i0q[5] = {0.4463598170691436, 0.001702205745042606, 2.792125684538934e-06, 2.369902034785866e-09,
                       8.965900179621208e-13};
const double i0pp[5] = {0.119227374812067, 0.1947452015979746, 0.07629241821600588, 0.00847490358080155,
                        0.0002023821945835647};
const double i0qq[6] = {0.2962898424533095, 0.4866115913196384, 0.1938352806477617,
                        0.02261671093400046, 0.0006450448095075585, 1.52983578240045e-06};

template <typename F>
void symmetric(int32_t N, double* hn, F f)
{
    for (int i = 0; i < (N + 1) / 2; i++) {
        double v = f(i);
        hn[i] = v;
        hn[N - 1 - i] = v;
    }
}

// Chebyshev polynomial of order n
double cheb(double x, int n)
{
    if (std::fabs(x) <= 1.0)
        return std::cos(n * std::acos(x));
    if (x <= 1.0)
        return ((n & 1) ? -1.0 : 1.0) * std::cosh(n * std::acosh(-x));
    return std::cosh(n * std::acosh(x));
}

// real part of the inverse DFT
void ridft(const std::vector<double>& re, const std::vector<double>& im, std::vector<double>& out)
{
    size_t n = re.size();
    out.assign(n, 0.0);
    for (size_t k = 0; k < n; k++) {
        double s = 0;
        for (size_t m = 0; m < n; m++) {
            double a = 2.0 * kPi * (double)((k * m) % n) / (double)n;
            s += re[m] * std::cos(a) - im[m] * std::sin(a);
        }
        out[k] = s / (double)n;
    }
}

} // namespace

double vmsig_poly(double x, const double* c, int n)
{
    double r = c[n];
    for (int i = n - 1; i >= 0; i--)
        r = r * x + c[i];
    return r;
}

double vmsig_bessel_i0(double x)
{
    double ax = std::fabs(x);
    if (ax < 15.0)
        return vmsig_poly(x * x, i0p, 13) / vmsig_poly(225.0 - x * x, i0q, 4);
    double t = 1.0 - 15.0 / ax;
    return std::exp(ax) * vmsig_poly(t, i0pp, 4) / (vmsig_poly(t, i0qq, 5) * std::sqrt(ax));
}

double vmsig_sinc(double x)
{
    if (x != 0.0)
        return std::sin(x * kPi) / (x * kPi);
    return 1.0;
}

double SIGNAL_DLL_CALL getdefaultpara(WINDOW_STYLE window)
{
    switch (window) {
    case TUKEY:
        return 0.2;
    case KAISER:
    case GAUSS:
        return 2.5;
    case DOLPH_CHEBYSHEV:
        return 30.0;
    default:
        return 0.0;
    }
}

void SIGNAL_DLL_CALL getpararange(WINDOW_STYLE window, double* min, double* max)
{
    switch (window) {
    case TUKEY:
        *min = 0.0;
        *max = 1.0;
        break;
    case KAISER:
        *min = 0.0;
        *max = 700.0;
        break;
    case GAUSS:
        *min = 2.0;
        *max = 50.0;
        break;
    case DOLPH_CHEBYSHEV:
        *min = 1.0;
        *max = 300.0;
        break;
    default:
        break;
    }
}

void SIGNAL_DLL_CALL rectangle(int32_t N, double* hn)
{
    symmetric(N, hn, [](int) { return 1.0; });
}

void SIGNAL_DLL_CALL bartlett(int32_t N, double* hn)
{
    symmetric(N, hn, [N](int i) { return (2.0 * i) / ((double)N - 1.0); });
}

void SIGNAL_DLL_CALL triangular(int32_t N, double* hn)
{
    if ((N & 1) == 0) {
        for (int i = 1; i <= N / 2; i++) {
            double v = (2.0 * i - 1.0) / (double)N;
            hn[i - 1] = v;
            hn[N - i] = v;
        }
    } else {
        for (int i = 1; i <= (N + 1) / 2; i++) {
            double v = (2.0 * i) / ((double)N + 1.0);
            hn[i - 1] = v;
            hn[N - i] = v;
        }
    }
}

void SIGNAL_DLL_CALL cosine(int32_t N, double* hn)
{
    symmetric(N, hn, [N](int i) { return std::cos((i * kPi) / (double)(N - 1) - kPi / 2.0); });
}

void SIGNAL_DLL_CALL hanning(int32_t N, double* hn)
{
    symmetric(N, hn, [N](int i) { return (1.0 - std::cos((2 * i * kPi) / (double)(N - 1))) * 0.5; });
}

void SIGNAL_DLL_CALL bartlett_hanning(int32_t N, double* hn)
{
    symmetric(N, hn, [N](int i) {
        double x = (double)i / (double)(N - 1);
        return (0.62 - std::fabs(x - 0.5) * 0.48) - std::cos(x * 2.0 * kPi) * 0.38;
    });
}

void SIGNAL_DLL_CALL hamming(int32_t N, double* hn)
{
    symmetric(N, hn, [N](int i) { return 0.54 - std::cos((2 * i * kPi) / (double)(N - 1)) * 0.46; });
}

void SIGNAL_DLL_CALL blackman(int32_t N, double* hn)
{
    symmetric(N, hn, [N](int i) {
        double d = (double)(N - 1);
        return (0.42 - std::cos((2 * i * kPi) / d) * 0.5) + std::cos((4 * i * kPi) / d) * 0.08;
    });
}

void SIGNAL_DLL_CALL blackman_Harris(int32_t N, double* hn)
{
    symmetric(N, hn, [N](int i) {
        double d = (double)(N - 1);
        return ((0.35875 - std::cos((2 * i * kPi) / d) * 0.48829) + std::cos((4 * i * kPi) / d) * 0.14128) -
               std::cos((6 * i * kPi) / d) * 0.01168;
    });
}

char SIGNAL_DLL_CALL tukey(int32_t N, double* hn, double alpha)
{
    if (!(alpha >= 0.0 && alpha <= 1.0))
        return 0;
    double w = (double)(N - 1) * alpha;
    int half = (N + 1) / 2;
    int i = 0;
    while (i < half) {
        while (w * 0.5 < (double)i) {
            hn[i] = 1.0;
            hn[N - 1 - i] = 1.0;
            i++;
            if (i >= half)
                return 1;
        }
        double v = (std::cos(((double)(i * 2) / w - 1.0) * kPi) + 1.0) * 0.5;
        hn[i] = v;
        hn[N - 1 - i] = v;
        i++;
    }
    return 0;
}

void SIGNAL_DLL_CALL Nuttall(int32_t N, double* hn)
{
    symmetric(N, hn, [N](int i) {
        double d = (double)(N - 1);
        return ((0.3635819 - std::cos((2 * i * kPi) / d) * 0.4891775) + std::cos((4 * i * kPi) / d) * 0.1365995) -
               std::cos((6 * i * kPi) / d) * 0.0106411;
    });
}

void SIGNAL_DLL_CALL FlatTop(int32_t N, double* hn)
{
    symmetric(N, hn, [N](int i) {
        double d = (double)(N - 1);
        return (((0.21557895 - std::cos((2 * i * kPi) / d) * 0.41663158) +
                 std::cos((4 * i * kPi) / d) * 0.277263158) -
                std::cos((6 * i * kPi) / d) * 0.083578947) +
               std::cos((8 * i * kPi) / d) * 0.006947368;
    });
}

void SIGNAL_DLL_CALL Bohman(int32_t N, double* hn)
{
    double c = (double)(N - 1) * 0.5;
    symmetric(N, hn, [c](int i) {
        double x = std::fabs(((double)i - c) / c);
        return std::sin(x * kPi) * 0.3183098861837907 + (1.0 - x) * std::cos(x * kPi);
    });
}

void SIGNAL_DLL_CALL Parzen(int32_t N, double* hn)
{
    double edge = (double)((1 - N) / 4);
    symmetric(N, hn, [N, edge](int i) {
        double n = (double)i - (double)(N - 1) * 0.5;
        double k = n / ((double)N * 0.5);
        double a = 1.0 - std::fabs(k);
        if (n < edge || n > 0.0)
            return 2.0 * a * a * a;
        return 1.0 - k * k * 6.0 * a;
    });
}

void SIGNAL_DLL_CALL Lanczos(int32_t N, double* hn)
{
    symmetric(N, hn, [N](int i) { return vmsig_sinc((2.0 * i) / ((double)N - 1.0) - 1.0); });
}

void SIGNAL_DLL_CALL kaiser(int32_t N, double* hn, double beta)
{
    double den = vmsig_bessel_i0(beta);
    symmetric(N, hn, [N, beta, den](int i) {
        double x = (2.0 * i) / (double)(N - 1) - 1.0;
        return vmsig_bessel_i0(std::sqrt(1.0 - x * x) * beta) / den;
    });
}

char SIGNAL_DLL_CALL gauss(int32_t N, double* hn, double alpha)
{
    if (!(alpha >= 2.0))
        return 0;
    double c = (double)(N - 1) * 0.5;
    symmetric(N, hn, [c, alpha](int i) {
        double x = (((double)i - c) * alpha) / c;
        return std::exp(x * x * -0.5);
    });
    return 1;
}

// Dolph-Chebyshev window, sidelobe attenuation 'at' dB (any N)
void SIGNAL_DLL_CALL dolph_chebyshev(int32_t N, double* hn, double at)
{
    if (N == 1) {
        hn[0] = 1.0;
        return;
    }
    if (N < 1)
        return;
    double x0 = std::cosh(std::acosh(1.0 / std::pow(10.0, -at / 20.0)) / (double)(N - 1));
    std::vector<double> re(N), im(N, 0.0), r;
    for (int k = 0; k < N; k++)
        re[k] = cheb(std::cos(k * kPi / (double)N) * x0, N - 1);
    int M = N / 2;
    if (N & 1) {
        ridft(re, im, r);
        for (int k = 0; k <= M; k++) {
            hn[k] = r[M - k];
            hn[N - 1 - k] = r[M - k];
        }
    } else {
        for (int k = 0; k < N; k++) {
            double p = re[k], a = -k * kPi / (double)N;
            re[k] = p * std::cos(a);
            im[k] = p * std::sin(a);
        }
        ridft(re, im, r);
        for (int k = 0; k < M; k++) {
            hn[k] = r[M - k];
            hn[N - 1 - k] = r[M - k];
        }
    }
    double mx = hn[0] < hn[M] ? hn[M] : hn[0];
    for (int k = 0; k < N; k++)
        hn[k] /= mx;
}

char SIGNAL_DLL_CALL dolph_chebyshev_fft(int32_t N, double* hn, double at)
{
    dolph_chebyshev(N, hn, at);
    return 1;
}

char SIGNAL_DLL_CALL window(WINDOW_STYLE type, int32_t N, double* hn, double alpha_beta, char normalize)
{
    char r = 1;
    switch (type) {
    case RECTANGLE: rectangle(N, hn); break;
    case BARTLETT: bartlett(N, hn); break;
    case TRIANGULAR: triangular(N, hn); break;
    case COSINE: cosine(N, hn); break;
    case HANNING: hanning(N, hn); break;
    case BARTLETT_HANNING: bartlett_hanning(N, hn); break;
    case HAMMING: hamming(N, hn); break;
    case BLACKMAN: blackman(N, hn); break;
    case BLACKMAN_HARRIS: blackman_Harris(N, hn); break;
    case TUKEY: r = tukey(N, hn, alpha_beta); break;
    case NUTTALL: Nuttall(N, hn); break;
    case FLATTOP: FlatTop(N, hn); break;
    case BOHMAN: Bohman(N, hn); break;
    case PARZEN: Parzen(N, hn); break;
    case LANCZOS: Lanczos(N, hn); break;
    case KAISER: kaiser(N, hn, alpha_beta); break;
    case GAUSS: r = gauss(N, hn, alpha_beta); break;
    case DOLPH_CHEBYSHEV: dolph_chebyshev(N, hn, alpha_beta); break;
    default: break;
    }
    if (normalize && N > 0) {
        double s = 0;
        for (int i = 0; i < N; i++)
            s += hn[i];
        for (int i = 0; i < N; i++)
            hn[i] /= s;
    }
    return r;
}

double SIGNAL_DLL_CALL window_i(WINDOW_STYLE type, double* x, int32_t N, double alpha_beta, char normalize)
{
    std::vector<double> w(N > 0 ? N : 0);
    window(type, N, w.data(), alpha_beta, normalize);
    double s = 0;
    for (int i = 0; i < N; i++) {
        s += w[i];
        x[i] *= w[i];
    }
    return s / (double)N;
}

double SIGNAL_DLL_CALL window_i2sum(WINDOW_STYLE type, int32_t N, double alpha_beta, char normalize)
{
    std::vector<double> w(N > 0 ? N : 0);
    window(type, N, w.data(), alpha_beta, normalize);
    double s = 0;
    for (int i = 0; i < N; i++)
        s += w[i] * w[i];
    return s;
}
