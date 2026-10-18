// Filtering of sample buffers (vmfilters.h). All variants compute the causal
// output y[i] = sum b[k] x[i-k] - sum a[k] y[i-k] (a[0] taken as 1) for the
// first 'num' samples, in place.
#include "vmfilters.h"

#include <cstring>
#include <vector>

#include "internal.h"

namespace {

template <typename T>
void ToDouble(const T* x, int32_t n, std::vector<double>& out)
{
    out.resize(n > 0 ? n : 0);
    for (int32_t i = 0; i < n; i++)
        out[i] = (double)x[i];
}

void FromDouble(const std::vector<double>& v, int32_t* x)
{
    for (size_t i = 0; i < v.size(); i++)
        x[i] = (int32_t)(long long)v[i];
}

void DirectFir(const double* h, int32_t N, double* x, int32_t num)
{
    std::vector<double> y(num > 0 ? num : 0, 0.0);
    for (int32_t i = 0; i < num; i++) {
        double s = 0;
        for (int32_t k = 0; k < N && k <= i; k++)
            s += x[i - k] * h[k];
        y[i] = s;
    }
    if (num > 0)
        std::memcpy(x, y.data(), num * sizeof(double));
}

void Recursive(const double* a, int32_t an, double* y, int32_t len)
{
    if (an < 2)
        return;
    for (int32_t j = 0; j < len; j++)
        for (int32_t m = 1; m < an && m <= j; m++)
            y[j] -= a[m] * y[j - m];
}

void DirectIir(const double* b, int32_t bn, const double* a, int32_t an, double* x, int32_t len)
{
    std::vector<double> y(len > 0 ? len : 0, 0.0);
    for (int32_t i = 0; i < len; i++) {
        double s = 0;
        for (int32_t k = 0; k < bn && k <= i; k++)
            s += x[i - k] * b[k];
        for (int32_t k = 1; k < an && k <= i; k++)
            s -= a[k] * y[i - k];
        y[i] = s;
    }
    if (len > 0)
        std::memcpy(x, y.data(), len * sizeof(double));
}

int32_t BlockFor(int32_t hn, int32_t num)
{
    int32_t n = 1;
    while (n < hn * 2 && n < (1 << 24))
        n <<= 1;
    (void)num;
    return n;
}

} // namespace

void SIGNAL_DLL_CALL filters(double* hn, int32_t N, double* date, int32_t num) { DirectFir(hn, N, date, num); }

void SIGNAL_DLL_CALL filters(double* hn, int32_t N, int32_t* date, int32_t num)
{
    std::vector<double> v;
    ToDouble(date, num, v);
    DirectFir(hn, N, v.data(), num);
    FromDouble(v, date);
}

void SIGNAL_DLL_CALL filters_fht(double* hn, int32_t N, double* date, int32_t fht_n)
{
    vmsig_convolve_inplace(hn, N, date, fht_n, fht_n);
}

void SIGNAL_DLL_CALL filters_fht(double* hn, int32_t N, int32_t* date, int32_t fht_n)
{
    std::vector<double> v;
    ToDouble(date, fht_n, v);
    filters_fht(hn, N, v.data(), fht_n);
    FromDouble(v, date);
}

void SIGNAL_DLL_CALL filters_fft(double* hn, int32_t N, double* date, int32_t fft_n)
{
    vmsig_convolve_inplace(hn, N, date, fft_n, fft_n);
}

void SIGNAL_DLL_CALL filters_fft(double* hn, int32_t N, int32_t* date, int32_t fft_n)
{
    std::vector<double> v;
    ToDouble(date, fft_n, v);
    filters_fft(hn, N, v.data(), fft_n);
    FromDouble(v, date);
}

void SIGNAL_DLL_CALL filters_overlap_add_fht(double* hn, int32_t N, double* date, int32_t num, int32_t fht_n)
{
    vmsig_convolve_inplace(hn, N, date, num, fht_n > N ? fht_n : BlockFor(N, num));
}

void SIGNAL_DLL_CALL filters_overlap_add_fht(double* hn, int32_t N, int32_t* date, int32_t num, int32_t fht_n)
{
    std::vector<double> v;
    ToDouble(date, num, v);
    filters_overlap_add_fht(hn, N, v.data(), num, fht_n);
    FromDouble(v, date);
}

void SIGNAL_DLL_CALL filters_overlap_add_fft(double* hn, int32_t N, double* date, int32_t num, int32_t fft_n)
{
    filters_overlap_add_fht(hn, N, date, num, fft_n);
}

void SIGNAL_DLL_CALL filters_overlap_add_fft(double* hn, int32_t N, int32_t* date, int32_t num, int32_t fft_n)
{
    filters_overlap_add_fht(hn, N, date, num, fft_n);
}

void SIGNAL_DLL_CALL filters(const std::vector<double> b, const std::vector<double> a, std::vector<double>& x,
                             int32_t len)
{
    std::vector<double> y(x);
    y.resize(len > 0 ? len : 0, 0.0);
    DirectIir(b.data(), (int32_t)b.size(), a.data(), (int32_t)a.size(), y.data(), len);
    x = y;
}

void SIGNAL_DLL_CALL filters(double* b, int32_t bn, double* a, int32_t an, double* x, int32_t len)
{
    DirectIir(b, bn, a, an, x, len);
}

void SIGNAL_DLL_CALL filters(double* b, int32_t bn, double* a, int32_t an, int32_t* x, int32_t len)
{
    std::vector<double> v;
    ToDouble(x, len, v);
    DirectIir(b, bn, a, an, v.data(), len);
    FromDouble(v, x);
}

void SIGNAL_DLL_CALL filters_overlap_add_fht(double* b, int32_t bn, double* a, int32_t an, double* x, int32_t len,
                                             int32_t fft_n)
{
    vmsig_convolve_inplace(b, bn, x, len, fft_n > bn ? fft_n : BlockFor(bn, len));
    Recursive(a, an, x, len);
}

void SIGNAL_DLL_CALL filters_overlap_add_fht(double* b, int32_t bn, double* a, int32_t an, int32_t* x, int32_t len,
                                             int32_t fft_n)
{
    std::vector<double> v;
    ToDouble(x, len, v);
    filters_overlap_add_fht(b, bn, a, an, v.data(), len, fft_n);
    FromDouble(v, x);
}
