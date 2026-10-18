// Helpers shared by the vmsignal implementation (not exported).
#pragma once

#include <complex>
#include <cstdint>
#include <vector>

#include "vmcplx_vector.h"

double vmsig_poly(double x, const double* c, int n);
double vmsig_bessel_i0(double x);
double vmsig_sinc(double x);

// In-place complex FFT of any length (radix-2 when n is a power of two,
// otherwise a direct DFT). sign = -1 forward, +1 inverse (unscaled).
void vmsig_fft(double* re, double* im, uint32_t n, int sign);

// Linear convolution y[i] = sum h[k] x[i-k] for i < num, computed in place
// with FFT overlap-add blocks of size fft_n.
void vmsig_convolve_inplace(const double* h, int32_t hn, double* x, int32_t num, int32_t fft_n);

// product of all elements (1 for an empty vector)
std::complex<double> vmsig_prod(const cplx_vector& v);
