// IIR filter design (iir/*.h): analog prototypes, frequency transforms,
// bilinear transform, order estimation, plus freqz and resp.
// 'z' as opt designs a digital filter (frequencies normalised to Nyquist).
#include "bilinear.h"
#include "butter.h"
#include "buttap.h"
#include "buttord.h"
#include "cheb1ap.h"
#include "cheb1ord.h"
#include "cheb2ap.h"
#include "cheb2ord.h"
#include "cheby1.h"
#include "cheby2.h"
#include "ellip.h"
#include "ellipap.h"
#include "ellipord.h"
#include "sftrans.h"
#include "vmcplx_fun.h"
#include "vmfilters.h"
#include "vmfreqz.h"
#include "vmresp.h"

#include <cmath>

#include "complete_elliptic_integral_first_kind.h"
#include "complex_jacobi_elliptic_functions.h"
#include "internal.h"
#include "inverse_jacobi_elliptic_functions.h"
#include "jacobi_elliptic_functions.h"
#include "jacobi_sn.h"

using cplx = std::complex<double>;

namespace {

const double kPi = 3.141592653589793;
const double k2OverPi = 0.6366197723675814;

double Warp(double w) { return std::tan(w * kPi * 0.5); }

// selectivity of the analog prototype for each band type
double Selectivity(int type, double wp1, double wp2, double ws1, double ws2)
{
    switch (type) {
    case LOW: return ws1 / wp1;
    case HIGH: return wp1 / ws1;
    case PASS: {
        double a = std::fabs((ws2 * ws2 - wp1 * wp2) / (ws2 * (wp1 - wp2)));
        double b = std::fabs((ws1 * ws1 - wp1 * wp2) / (ws1 * (wp1 - wp2)));
        return b <= a ? b : a;
    }
    case STOP: {
        double a = std::fabs(((wp2 - wp1) * ws2) / (ws2 * ws2 - wp1 * wp2));
        double b = std::fabs(((wp2 - wp1) * ws1) / (ws1 * ws1 - wp1 * wp2));
        return b <= a ? b : a;
    }
    default: return 1.0;
    }
}

// selectivity k -> transformed k1 (Orfanidis ellipdeg and its inverse)
double Cal_k1tok(int n, double k1)
{
    double k1p = std::sqrt(1.0 - k1 * k1);
    double K = Complete_Elliptic_Integral_First_Kind('k', k1p);
    double kp = std::pow(k1p, (double)n);
    for (int i = 1; i <= n / 2; i++)
        kp *= std::pow(Jacobi_sn(((double)(2 * i - 1) / (double)n) * K, 'k', k1p), 4.0);
    return std::sqrt(1.0 - kp * kp);
}

int Cal_elliptic_N_basic(double ratio, double rp, double rs)
{
    if (rs > 160.0)
        return -2;
    double k = 1.0 / ratio;
    double k1 = std::sqrt(std::pow(10.0L, 0.1L * rp) - 1.0L) / std::sqrt(std::pow(10.0L, 0.1L * rs) - 1.0L);
    double K = Complete_Elliptic_Integral_First_Kind('k', k);
    double Kp = Complete_Elliptic_Integral_First_Kind('k', std::sqrt(1.0 - k * k));
    double K1 = Complete_Elliptic_Integral_First_Kind('k', k1);
    double K1p = Complete_Elliptic_Integral_First_Kind('k', std::sqrt(1.0 - k1 * k1));
    return (int)(((K1p * K) / K1) / Kp + 1.0);
}

template <typename Proto>
void Design(char opt, double wn1, double wn2, BANDTYPE type, Proto proto, cplx_vector& zz, cplx_vector& zp,
            double& zg)
{
    if (opt == 'z') {
        wn1 = Warp(wn1);
        wn2 = Warp(wn2);
    }
    cplx_vector z, p, sz, sp;
    double k = 1.0, sg = 1.0;
    proto(z, p, k);
    sftrans(type, z, p, k, wn1, wn2, sz, sp, sg);
    if (opt != 'z') {
        zz = sz;
        zp = sp;
        zg = sg;
        return;
    }
    bilinear(sz, sp, sg, 2.0, zz, zp, zg);
}

std::vector<cplx> EvalFreq(const std::vector<cplx>& b, const std::vector<cplx>& a, int len)
{
    std::vector<cplx> h(len > 0 ? len : 0);
    for (int i = 0; i < len; i++) {
        double w = ((double)i * -kPi) / (double)(len - 1);
        cplx nb = 0, na = 0;
        for (size_t m = 0; m < b.size(); m++)
            nb += b[m] * std::polar(1.0, w * (double)m);
        for (size_t m = 0; m < a.size(); m++)
            na += a[m] * std::polar(1.0, w * (double)m);
        h[i] = nb / na;
    }
    return h;
}

std::vector<double> Present(const std::vector<cplx>& h, int sign)
{
    int len = (int)h.size();
    std::vector<double> r(len, 0.0);
    if (sign == 0) {
        for (int i = 0; i < len; i++)
            r[i] = std::abs(h[i]);
    } else if (sign == 1) {
        for (int i = 0; i < len; i++) {
            double a = std::abs(h[i]);
            if (a == 0.0)
                r[i] = i == 0 ? -800.0 : r[i - 1];
            else
                r[i] = std::log10(a) * 20.0;
        }
    } else if (sign == 2) {
        double off = 0.0;
        for (int i = 0; i < len; i++) {
            if (std::abs(h[i]) < 1e-10) {
                r[i] = i == 0 ? 0.0 : r[i - 1];
                continue;
            }
            double ph = std::atan2(h[i].imag(), h[i].real());
            double v = off + ph;
            if (i != 0) {
                double d = r[i - 1] - v;
                if (d < -6.2) {
                    off -= 2.0 * kPi;
                    v = ph + off;
                } else if (d > 6.2) {
                    off += 2.0 * kPi;
                    v = ph + off;
                }
            }
            r[i] = v;
        }
    }
    return r;
}

std::vector<cplx> PolyFromRoots(const cplx_vector& r)
{
    std::vector<cplx> c(1, 1.0);
    for (const cplx& x : r) {
        c.push_back(0.0);
        for (size_t i = c.size() - 1; i > 0; i--)
            c[i] -= x * c[i - 1];
    }
    return c;
}

} // namespace

std::complex<double> cplx_prod(cplx_vector c) { return vmsig_prod(c); }

std::complex<double> vmsig_prod(const cplx_vector& v)
{
    cplx r = 1.0;
    for (const cplx& x : v)
        r *= x;
    return r;
}

// ------------------------------------------------------------ prototypes

void SIGNAL_DLL_CALL buttap(int32_t n, cplx_vector& p, double& k)
{
    p.resize(n > 0 ? n : 0);
    for (int i = 0; i < n / 2; i++) {
        double a = (((double)(2 * i) + 1.0) / (double)(n * 2) + 0.5) * kPi;
        cplx v = std::exp(cplx(0.0, a));
        p[2 * i] = v;
        p[2 * i + 1] = std::conj(v);
    }
    if (n & 1)
        p[n - 1] = -1.0;
    k = 1.0;
}

void SIGNAL_DLL_CALL cheb1ap(int32_t n, double rp, cplx_vector& p, double& k)
{
    double eps = std::sqrt(std::pow(10.0, rp * 0.1) - 1.0);
    double v0 = std::asinh(1.0 / eps) / (double)n;
    p.resize(n > 0 ? n : 0);
    for (int i = 0; i < n / 2; i++) {
        double t = (((double)(2 * i + 2) - 1.0) * kPi) / (double)(2 * n);
        double re = -std::sinh(v0) * std::sin(t), im = std::cos(t) * std::cosh(v0);
        p[2 * i] = cplx(re, im);
        p[2 * i + 1] = cplx(re, -im);
    }
    if (n & 1) {
        double t = (((double)(2 * (n / 2) + 2) - 1.0) * kPi) / (double)(2 * n);
        p[n - 1] = cplx(-(std::sinh(v0) * std::sin(t)), 0.0);
    }
    cplx_vector np = -p;
    k = vmsig_prod(np).real();
    if (!(n & 1))
        k /= std::sqrt(eps * eps + 1.0);
}

void SIGNAL_DLL_CALL cheb2ap(int32_t n, double rs, cplx_vector& z, cplx_vector& p, double& k)
{
    double eps = 1.0 / std::sqrt(std::pow(10.0, rs * 0.1) - 1.0);
    double v0 = std::asinh(1.0 / eps) / (double)n;
    p.resize(n > 0 ? n : 0);
    for (int i = 0; i < n / 2; i++) {
        double t = (((double)(2 * i + 2) - 1.0) * kPi) / (double)(2 * n);
        double re = -(std::sinh(v0) * std::sin(t)), im = std::cos(t) * std::cosh(v0);
        double d = im * im + re * re;
        p[2 * i] = cplx(re / d, im / d);
        p[2 * i + 1] = cplx(re / d, -im / d);
    }
    if (n & 1) {
        double t = (((double)(2 * (n / 2) + 2) - 1.0) * kPi) / (double)(2 * n);
        p[n - 1] = cplx(1.0 / -(std::sinh(v0) * std::sin(t)), 0.0);
        z.resize(n - 1);
    } else {
        z.resize(n > 0 ? n : 0);
    }
    for (int i = 0; i < n / 2; i++) {
        double c = std::cos((((double)(2 * i + 2) - 1.0) * kPi) / (double)(2 * n));
        z[2 * i] = cplx(0.0, 1.0 / c);
        z[2 * i + 1] = cplx(0.0, -(1.0 / c));
    }
    cplx_vector nz = -z, np = -p;
    k = vmsig_prod(np).real() / vmsig_prod(nz).real();
}

void SIGNAL_DLL_CALL ellipap(int32_t n, double rp, double rs, cplx_vector& z, cplx_vector& p, double& k)
{
    double ep = std::sqrt(std::pow(10.0, rp * 0.1) - 1.0);
    double es = std::sqrt(std::pow(10.0, rs * 0.1) - 1.0);
    double k1 = ep / es;
    double K1 = Complete_Elliptic_Integral_First_Kind('k', k1);
    double km = Cal_k1tok(n, k1);
    double K = Complete_Elliptic_Integral_First_Kind('k', km);
    double w = Inverse_Jacobi_cn(ep / std::sqrt(ep * ep + 1.0), 'k', std::sqrt(1.0 - k1 * k1));
    double v0 = (w / (double)n) / K1;
    p.resize(n > 0 ? n : 0);
    z.resize((n & 1) ? n - 1 : n);
    for (int i = 0; i < n / 2; i++) {
        double u = ((double)(2 * i + 1) / (double)n) * K;
        double sd, cd, nd;
        Jacobi_sd_cd_nd(u, 'k', km, &sd, &cd, &nd);
        double zi = 1.0 / (km * cd);
        z[2 * i] = cplx(0.0, zi);
        z[2 * i + 1] = cplx(0.0, -zi);
        double cre, cim;
        complex_Jacobi_cd(u, K * -v0, 'k', km, &cre, &cim);
        p[2 * i] = cplx(-cim, cre);
        p[2 * i + 1] = cplx(-cim, -cre);
    }
    double h0;
    if (n & 1) {
        double u = ((double)(2 * (n / 2 + 1) - 1) / (double)n) * K;
        double cre, cim;
        complex_Jacobi_cd(u, K * -v0, 'k', km, &cre, &cim);
        p[n - 1] = cplx(-cim, cre);
        h0 = 1.0;
    } else {
        h0 = std::pow(10.0, -rp / 20.0);
    }
    cplx_vector nz = -z, np = -p;
    k = std::fabs((h0 * vmsig_prod(np) / vmsig_prod(nz)).real());
}

// ------------------------------------------------------ s-plane transforms

char SIGNAL_DLL_CALL sftrans_lp(cplx_vector z, cplx_vector p, double g, double w, cplx_vector& sz, cplx_vector& sp,
                                double& sg)
{
    sg = g * std::pow(1.0 / w, (double)z.size() - (double)p.size());
    sp = p * cplx(w, 0.0);
    sz = z * cplx(w, 0.0);
    return 1;
}

char SIGNAL_DLL_CALL sftrans_hp(cplx_vector z, cplx_vector p, double g, double w, cplx_vector& sz, cplx_vector& sp,
                                double& sg)
{
    cplx_vector nz = -z, np = -p;
    if (z.empty())
        sg = g * (1.0 / vmsig_prod(np)).real();
    else if (p.empty())
        sg = g * vmsig_prod(nz).real();
    else
        sg = g * (vmsig_prod(nz) / vmsig_prod(np)).real();
    sp.resize(p.size());
    for (size_t i = 0; i < p.size(); i++)
        sp[i] = w / p[i];
    sz.clear();
    for (size_t i = 0; i < z.size(); i++)
        sz.push_back(w / z[i]);
    while (sz.size() < p.size())
        sz.push_back(0.0);
    return 1;
}

char SIGNAL_DLL_CALL sftrans_bp(cplx_vector z, cplx_vector p, double g, double w0, double w1, cplx_vector& sz,
                                cplx_vector& sp, double& sg)
{
    double fl = w0, fh = w1;
    sg = g * std::pow(1.0 / (fh - fl), (double)z.size() - (double)p.size());
    auto split = [&](const cplx_vector& r, cplx_vector& out) {
        out.clear();
        std::vector<cplx> b(r.size());
        for (size_t i = 0; i < r.size(); i++)
            b[i] = r[i] * ((fh - fl) / 2.0);
        for (size_t i = 0; i < r.size(); i++)
            out.push_back(b[i] + std::sqrt(b[i] * b[i] - fh * fl));
        for (size_t i = 0; i < r.size(); i++)
            out.push_back(b[i] - std::sqrt(b[i] * b[i] - fh * fl));
    };
    split(p, sp);
    if (z.empty()) {
        sz.assign(p.size(), 0.0);
    } else {
        split(z, sz);
        for (size_t i = z.size(); i < p.size(); i++)
            sz.push_back(0.0);
    }
    return 1;
}

char SIGNAL_DLL_CALL sftrans_sb(cplx_vector z, cplx_vector p, double g, double w0, double w1, cplx_vector& sz,
                                cplx_vector& sp, double& sg)
{
    double fl = w0, fh = w1;
    cplx_vector nz = -z, np = -p;
    if (z.empty())
        sg = g * (1.0 / vmsig_prod(np)).real();
    else if (p.empty())
        sg = g * vmsig_prod(nz).real();
    else
        sg = g * (vmsig_prod(nz) / vmsig_prod(np)).real();
    auto split = [&](const cplx_vector& r, cplx_vector& out) {
        out.clear();
        std::vector<cplx> b(r.size());
        for (size_t i = 0; i < r.size(); i++)
            b[i] = ((fh - fl) / 2.0) / r[i];
        for (size_t i = 0; i < r.size(); i++)
            out.push_back(b[i] + std::sqrt(b[i] * b[i] - fh * fl));
        for (size_t i = 0; i < r.size(); i++)
            out.push_back(b[i] - std::sqrt(b[i] * b[i] - fh * fl));
    };
    split(p, sp);
    cplx ext = std::sqrt(cplx(-fh * fl, 0.0));
    if (z.empty()) {
        sz.clear();
        for (size_t i = 1; i <= 2 * p.size(); i++)
            sz.push_back((i & 1) ? -ext : ext);
    } else {
        split(z, sz);
        for (size_t i = 1; i <= 2 * (p.size() - z.size()); i++)
            sz.push_back((i & 1) ? -ext : ext);
    }
    return 1;
}

char SIGNAL_DLL_CALL sftrans(BANDTYPE type, cplx_vector z, cplx_vector p, double g, double w0, double w1,
                             cplx_vector& sz, cplx_vector& sp, double& sg)
{
    if (z.size() > p.size() || p.empty())
        return 0;
    switch (type) {
    case LOW: return sftrans_lp(z, p, g, w0, sz, sp, sg);
    case HIGH: return sftrans_hp(z, p, g, w0, sz, sp, sg);
    case PASS: return sftrans_bp(z, p, g, w0, w1, sz, sp, sg);
    case STOP: return sftrans_sb(z, p, g, w0, w1, sz, sp, sg);
    default: return 0;
    }
}

bool SIGNAL_DLL_CALL bilinear(cplx_vector z, cplx_vector p, double g, double T, cplx_vector& Zz, cplx_vector& Zp,
                              double& Zg)
{
    if (z.size() > p.size() || p.empty())
        return false;
    cplx num = 1.0, den = 1.0;
    for (const cplx& x : z)
        num *= (2.0 - x * T) / T;
    for (const cplx& x : p)
        den *= (2.0 - x * T) / T;
    Zg = (g * num / den).real();
    Zp.resize(p.size());
    for (size_t i = 0; i < p.size(); i++)
        Zp[i] = (2.0 + p[i] * T) / (2.0 - p[i] * T);
    Zz.clear();
    for (const cplx& x : z)
        Zz.push_back((2.0 + x * T) / (2.0 - x * T));
    while (Zz.size() < Zp.size())
        Zz.push_back(-1.0);
    return true;
}

// ------------------------------------------------------------- designs

void SIGNAL_DLL_CALL butter(int32_t n, double wn1, double wn2, BANDTYPE type, char opt, cplx_vector& zz,
                            cplx_vector& zp, double& zg)
{
    Design(opt, wn1, wn2, type, [n](cplx_vector&, cplx_vector& p, double& k) { buttap(n, p, k); }, zz, zp, zg);
}

void SIGNAL_DLL_CALL cheby1(int32_t n, double wn1, double wn2, double rp, BANDTYPE type, char opt, cplx_vector& zz,
                            cplx_vector& zp, double& zg)
{
    Design(opt, wn1, wn2, type, [n, rp](cplx_vector&, cplx_vector& p, double& k) { cheb1ap(n, rp, p, k); }, zz, zp,
           zg);
}

void SIGNAL_DLL_CALL cheby2(int32_t n, double wn1, double wn2, double rs, BANDTYPE type, char opt, cplx_vector& zz,
                            cplx_vector& zp, double& zg)
{
    Design(opt, wn1, wn2, type, [n, rs](cplx_vector& z, cplx_vector& p, double& k) { cheb2ap(n, rs, z, p, k); },
           zz, zp, zg);
}

void SIGNAL_DLL_CALL ellip(int32_t n, double wn1, double wn2, double rp, double rs, BANDTYPE type, char opt,
                           cplx_vector& zz, cplx_vector& zp, double& zg)
{
    Design(opt, wn1, wn2, type,
           [n, rp, rs](cplx_vector& z, cplx_vector& p, double& k) { ellipap(n, rp, rs, z, p, k); }, zz, zp, zg);
}

// --------------------------------------------------------- order estimation

int32_t SIGNAL_DLL_CALL buttord(BANDTYPE type, double wp1, double wp2, double ws1, double ws2, double rp, double rs,
                                char opt, double& wn1, double& wn2)
{
    if (opt == 'z') {
        wp1 = Warp(wp1);
        wp2 = Warp(wp2);
        ws1 = Warp(ws1);
        ws2 = Warp(ws2);
    }
    double r = Selectivity(type, wp1, wp2, ws1, ws2);
    int n = (int)(std::log10((std::pow(10.0, rs * 0.1) - 1.0) / (std::pow(10.0, rp * 0.1) - 1.0)) /
                      (2.0 * std::log10(r)) +
                  1.0);
    double w0 = r / std::pow(std::pow(10.0, rs * 0.1) - 1.0, 1.0 / (2.0 * (double)n));
    if (type == PASS) {
        double d = wp2 - wp1;
        double q = std::sqrt(wp1 * wp2 + w0 * w0 * 0.25 * d * d);
        wn1 = -w0 * d * 0.5 + q;
        wn2 = w0 * d * 0.5 + q;
    } else if (type == STOP) {
        double d = wp2 - wp1;
        double q = std::sqrt(wp1 * w0 * 4.0 * w0 * wp2 + d * d);
        wn1 = (d + q) / (2.0 * w0);
        wn2 = (d - q) / (2.0 * w0);
    } else {
        if (type == LOW)
            wn1 = wp1 * w0;
        else if (type == HIGH)
            wn1 = wp1 / w0;
        goto unwarp;
    }
    wn1 = std::fabs(wn1);
    wn2 = std::fabs(wn2);
    if (wn2 < wn1)
        std::swap(wn1, wn2);
unwarp:
    if (opt == 'z') {
        wn1 = std::atan(wn1) * k2OverPi;
        wn2 = std::atan(wn2) * k2OverPi;
    }
    return n;
}

int32_t SIGNAL_DLL_CALL cheb1ord(BANDTYPE type, double wp1, double wp2, double ws1, double ws2, double rp, double rs,
                                 char opt, double& wn1, double& wn2)
{
    if (opt == 'z') {
        wp1 = Warp(wp1);
        wp2 = Warp(wp2);
        ws1 = Warp(ws1);
        ws2 = Warp(ws2);
    }
    double r = Selectivity(type, wp1, wp2, ws1, ws2);
    double a = std::acosh(std::sqrt((std::pow(10.0, rs * 0.1) - 1.0) / (std::pow(10.0, rp * 0.1) - 1.0)));
    double b = std::acosh(r);
    wn1 = wp1;
    wn2 = wp2;
    if (opt == 'z') {
        wn1 = std::atan(wn1) * k2OverPi;
        wn2 = std::atan(wn2) * k2OverPi;
    }
    return (int)(a / b + 1.0);
}

int32_t SIGNAL_DLL_CALL cheb2ord(BANDTYPE type, double wp1, double wp2, double ws1, double ws2, double rp, double rs,
                                 char opt, double& wn1, double& wn2)
{
    const double ows1 = ws1, ows2 = ws2;
    if (opt == 'z') {
        wp1 = Warp(wp1);
        wp2 = Warp(wp2);
        ws1 = Warp(ws1);
        ws2 = Warp(ws2);
    }
    double r = Selectivity(type, wp1, wp2, ws1, ws2);
    double e = std::acosh(std::sqrt((std::pow(10.0, rs * 0.1) - 1.0) / (std::pow(10.0, rp * 0.1) - 1.0)));
    int n = (int)(e / std::acosh(r) + 1.0);
    double w = 1.0 / std::cosh(e * (1.0 / (double)n));
    if (type == PASS) {
        double q = std::sqrt(((wp2 - wp1) * (wp2 - wp1)) / (w * w * 4.0) + wp1 * wp2);
        wn1 = (wp1 - wp2) / (2.0 * w) + q;
        wn2 = (wp1 * wp2) / wn1;
    } else if (type == LOW) {
        wn1 = wp1 / w;
    } else if (type == HIGH) {
        wn1 = w * wp1;
    } else if (type == STOP) {
        double q = std::sqrt(w * (wp2 - wp1) * (wp2 - wp1) * w * 0.25 + wp1 * wp2);
        wn1 = (wp1 - wp2) * w * 0.5 + q;
        wn2 = (wp1 * wp2) / wn1;
    }
    if (opt == 'z') {
        // the vendor library returns the stop band edges for digital designs
        wn1 = ows1;
        wn2 = ows2;
    }
    return n;
}

int32_t SIGNAL_DLL_CALL ellipord(BANDTYPE type, double wp1, double wp2, double ws1, double ws2, double rp, double rs,
                                 char opt, double& wn1, double& wn2)
{
    if (opt == 'z') {
        wp1 = Warp(wp1);
        wp2 = Warp(wp2);
        ws1 = Warp(ws1);
        ws2 = Warp(ws2);
    }
    int n = 0;
    if (type <= STOP)
        n = Cal_elliptic_N_basic(Selectivity(type, wp1, wp2, ws1, ws2), rp, rs);
    wn1 = std::atan(wp1) * k2OverPi;
    wn2 = std::atan(wp2) * k2OverPi;
    return n;
}

// ------------------------------------------------------ responses

std::vector<double> SIGNAL_DLL_CALL freqz(const std::vector<double> b, const std::vector<double> a, int32_t len,
                                          int32_t sign)
{
    std::vector<cplx> cb(b.begin(), b.end()), ca(a.begin(), a.end());
    return Present(EvalFreq(cb, ca, len), sign);
}

std::vector<double> SIGNAL_DLL_CALL freqz(const cplx_vector b, const cplx_vector a, int32_t len, int32_t sign)
{
    std::vector<cplx> cb(b.begin(), b.end()), ca(a.begin(), a.end());
    return Present(EvalFreq(cb, ca, len), sign);
}

std::vector<double> SIGNAL_DLL_CALL freqz(const cplx_vector z, const cplx_vector p, double g, int32_t len,
                                          int32_t sign)
{
    std::vector<cplx> cb = PolyFromRoots(z), ca = PolyFromRoots(p);
    for (cplx& c : cb)
        c *= g;
    return Present(EvalFreq(cb, ca, len), sign);
}

// step (morj = 0) or impulse (morj = 1) response
std::vector<double> SIGNAL_DLL_CALL resp(const std::vector<double> b, const std::vector<double> a, int32_t len,
                                         char morj)
{
    std::vector<double> x(len > 0 ? len : 0, morj == 1 ? 0.0 : 1.0);
    if (morj == 1 && len > 0)
        x[0] = 1.0;
    filters(b, a, x, len);
    return x;
}
