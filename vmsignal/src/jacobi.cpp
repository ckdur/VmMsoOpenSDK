// Jacobi elliptic functions and elliptic integrals (jacobi/*.h).
// arg selects the meaning of x: 'k' modulus, 'm' parameter (k^2), 'a' modular angle.
#include "complete_elliptic_integral_first_kind.h"
#include "complex_jacobi_elliptic_functions.h"
#include "inverse_jacobi_elliptic_functions.h"
#include "jacobi_cn.h"
#include "jacobi_dn.h"
#include "jacobi_elliptic_functions.h"
#include "jacobi_sn.h"

#include <cmath>
#include <complex>

namespace {

const double kPi = 3.141592653589793;

double ParamM(char arg, double x)
{
    switch (arg) {
    case 'k': return x * x;
    case 'a': {
        double s = std::sin(x);
        return s * s;
    }
    default: return x;
    }
}

void SnCnDn(double u, double m, double* sn, double* cn, double* dn)
{
    if (m < 1e-16) {
        *sn = std::sin(u);
        *cn = std::cos(u);
        *dn = 1.0;
        return;
    }
    if (m >= 1.0 - 1e-16) {
        double t = std::tanh(u), s = 1.0 / std::cosh(u);
        *sn = t;
        *cn = s;
        *dn = s;
        return;
    }
    // arithmetic-geometric mean (Abramowitz & Stegun 16.4)
    const int kMax = 32;
    double a[kMax + 1], c[kMax + 1];
    a[0] = 1.0;
    double b = std::sqrt(1.0 - m);
    c[0] = std::sqrt(m);
    int n = 0;
    while (n < kMax && std::fabs(c[n]) > 1e-16 * a[n]) {
        double an = a[n];
        a[n + 1] = (an + b) * 0.5;
        c[n + 1] = (an - b) * 0.5;
        b = std::sqrt(an * b);
        n++;
    }
    double phi = std::ldexp(a[n] * u, n);
    double phi1 = phi;
    for (int i = n; i > 0; i--) {
        phi1 = phi;
        phi = (phi + std::asin(c[i] * std::sin(phi) / a[i])) * 0.5;
    }
    *sn = std::sin(phi);
    *cn = std::cos(phi);
    *dn = n > 0 ? std::cos(phi) / std::cos(phi1 - phi) : std::sqrt(1.0 - m * (*sn) * (*sn));
}

// Carlson symmetric integral R_F
double CarlsonRF(double x, double y, double z)
{
    for (int i = 0; i < 100; i++) {
        double sx = std::sqrt(x), sy = std::sqrt(y), sz = std::sqrt(z);
        double l = sx * (sy + sz) + sy * sz;
        x = (x + l) * 0.25;
        y = (y + l) * 0.25;
        z = (z + l) * 0.25;
        double mu = (x + y + z) / 3.0;
        double dx = 1.0 - x / mu, dy = 1.0 - y / mu, dz = 1.0 - z / mu;
        double e = std::fmax(std::fabs(dx), std::fmax(std::fabs(dy), std::fabs(dz)));
        if (e < 1e-6) {
            double e2 = dx * dy - dz * dz, e3 = dx * dy * dz;
            return (1.0 + (e2 / 24.0 - 0.1 - 3.0 * e3 / 44.0) * e2 + e3 / 14.0) / std::sqrt(mu);
        }
    }
    return 1.0 / std::sqrt((x + y + z) / 3.0);
}

double EllipticF(double phi, double m)
{
    // reduce to [-pi/2, pi/2] using quasi periodicity
    double K = Complete_Elliptic_Integral_First_Kind('m', m);
    double n = std::floor(phi / kPi + 0.5);
    double p = phi - n * kPi;
    double s = std::sin(p), c = std::cos(p);
    return s * CarlsonRF(c * c, 1.0 - m * s * s, 1.0) + 2.0 * n * K;
}

} // namespace

double Complete_Elliptic_Integral_First_Kind(char arg, double x)
{
    double m = ParamM(arg, x);
    if (m >= 1.0)
        return HUGE_VAL;
    double a = 1.0, b = std::sqrt(1.0 - m);
    for (int i = 0; i < 64 && std::fabs(a - b) > 1e-16 * a; i++) {
        double t = (a + b) * 0.5;
        b = std::sqrt(a * b);
        a = t;
    }
    return kPi / (2.0 * a);
}

double Legendre_Elliptic_Integral_First_Kind(double amplitude, char arg, double x)
{
    return EllipticF(amplitude, ParamM(arg, x));
}

void Jacobi_sn_cn_dn(double u, char arg, double x, double* sn, double* cn, double* dn)
{
    SnCnDn(u, ParamM(arg, x), sn, cn, dn);
}

double Jacobi_sn(double u, char arg, double x)
{
    double s, c, d;
    SnCnDn(u, ParamM(arg, x), &s, &c, &d);
    return s;
}

double Jacobi_cn(double u, char arg, double x)
{
    double s, c, d;
    SnCnDn(u, ParamM(arg, x), &s, &c, &d);
    return c;
}

double Jacobi_dn(double u, char arg, double x)
{
    double s, c, d;
    SnCnDn(u, ParamM(arg, x), &s, &c, &d);
    return d;
}

void Jacobi_cs_ds_ns(double u, char arg, double x, double* cs, double* ds, double* ns)
{
    double s, c, d;
    SnCnDn(u, ParamM(arg, x), &s, &c, &d);
    *cs = c / s;
    *ds = d / s;
    *ns = 1.0 / s;
}

void Jacobi_sc_dc_nc(double u, char arg, double x, double* sc, double* dc, double* nc)
{
    double s, c, d;
    SnCnDn(u, ParamM(arg, x), &s, &c, &d);
    *sc = s / c;
    *dc = d / c;
    *nc = 1.0 / c;
}

void Jacobi_sd_cd_nd(double u, char arg, double x, double* sd, double* cd, double* nd)
{
    double s, c, d;
    SnCnDn(u, ParamM(arg, x), &s, &c, &d);
    *sd = s / d;
    *cd = c / d;
    *nd = 1.0 / d;
}

namespace {
// sn, cn, dn of the complex argument ure + j*uim
void ComplexSnCnDn(double ure, double uim, double m, std::complex<double>* sn, std::complex<double>* cn,
                   std::complex<double>* dn)
{
    double s, c, d, s1, c1, d1;
    SnCnDn(ure, m, &s, &c, &d);
    SnCnDn(uim, 1.0 - m, &s1, &c1, &d1);
    double den = c1 * c1 + m * s * s * s1 * s1;
    *sn = std::complex<double>(s * d1, c * d * s1 * c1) / den;
    *cn = std::complex<double>(c * c1, -s * d * s1 * d1) / den;
    *dn = std::complex<double>(d * c1 * d1, -m * s * c * s1) / den;
}
} // namespace

void complex_Jacobi_sn(double ure, double uim, char arg, double x, double* snre, double* snim)
{
    std::complex<double> s, c, d;
    ComplexSnCnDn(ure, uim, ParamM(arg, x), &s, &c, &d);
    *snre = s.real();
    *snim = s.imag();
}

void complex_Jacobi_cn(double ure, double uim, char arg, double x, double* cnre, double* cnim)
{
    std::complex<double> s, c, d;
    ComplexSnCnDn(ure, uim, ParamM(arg, x), &s, &c, &d);
    *cnre = c.real();
    *cnim = c.imag();
}

void complex_Jacobi_dn(double ure, double uim, char arg, double x, double* dnre, double* dnim)
{
    std::complex<double> s, c, d;
    ComplexSnCnDn(ure, uim, ParamM(arg, x), &s, &c, &d);
    *dnre = d.real();
    *dnim = d.imag();
}

void complex_Jacobi_cd(double ure, double uim, char arg, double x, double* cdre, double* cdim)
{
    std::complex<double> s, c, d;
    ComplexSnCnDn(ure, uim, ParamM(arg, x), &s, &c, &d);
    std::complex<double> r = c / d;
    *cdre = r.real();
    *cdim = r.imag();
}

double Inverse_Jacobi_sn(double x, char arg, double param)
{
    return EllipticF(std::asin(x), ParamM(arg, param));
}

double Inverse_Jacobi_cn(double x, char arg, double param)
{
    return EllipticF(std::acos(x), ParamM(arg, param));
}

double Inverse_Jacobi_dn(double x, char arg, double param)
{
    double m = ParamM(arg, param);
    double s2 = (1.0 - x * x) / m;
    return EllipticF(std::asin(std::sqrt(s2 < 0 ? 0 : (s2 > 1 ? 1 : s2))), m);
}
