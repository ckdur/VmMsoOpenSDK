// Numerical helpers (vmsuanfa.h): QR decomposition, eigenvalues of an upper
// Hessenberg matrix and polynomial roots.
#include "vmsuanfa.h"

#include <cmath>
#include <vector>

// Householder QR of the m x n matrix a (m >= n): q receives Q (m x m), a is
// replaced by R. Returns 0 if m < n.
int32_t SIGNAL_DLL_CALL maqr(int32_t m, int32_t n, double* a, double* q)
{
    if (m < n)
        return 0;
    for (int i = 0; i < m; i++)
        for (int j = 0; j < m; j++)
            q[i * m + j] = i == j ? 1.0 : 0.0;
    int nn = n == m ? n - 1 : n;
    for (int k = 0; k < nn; k++) {
        double u = 0;
        for (int i = k; i < m; i++)
            u = std::fmax(u, std::fabs(a[i * n + k]));
        double alpha = 0;
        for (int i = k; i < m; i++) {
            double t = a[i * n + k] / u;
            alpha += t * t;
        }
        if (a[k * n + k] > 0)
            u = -u;
        alpha = u * std::sqrt(alpha);
        if (std::fabs(alpha) == 0)
            return 0;
        u = std::sqrt(2.0 * alpha * (alpha - a[k * n + k]));
        if (u != 0) {
            a[k * n + k] = (a[k * n + k] - alpha) / u;
            for (int i = k + 1; i < m; i++)
                a[i * n + k] /= u;
            for (int j = 0; j < m; j++) {
                double t = 0;
                for (int l = k; l < m; l++)
                    t += a[l * n + k] * q[l * m + j];
                for (int i = k; i < m; i++)
                    q[i * m + j] -= 2.0 * t * a[i * n + k];
            }
            for (int j = k + 1; j < n; j++) {
                double t = 0;
                for (int l = k; l < m; l++)
                    t += a[l * n + k] * a[l * n + j];
                for (int i = k; i < m; i++)
                    a[i * n + j] -= 2.0 * t * a[i * n + k];
            }
            a[k * n + k] = alpha;
            for (int i = k + 1; i < m; i++)
                a[i * n + k] = 0;
        }
    }
    // q holds Q^T, transpose it
    for (int i = 0; i < m - 1; i++)
        for (int j = i + 1; j < m; j++)
            std::swap(q[i * m + j], q[j * m + i]);
    return 1;
}

// Eigenvalues of the n x n upper Hessenberg matrix a (double shift QR).
// Returns 1 on success, -1 if more than jt iterations are needed for a root.
int32_t SIGNAL_DLL_CALL qrtt(int32_t n, double a[], double u[], double v[], double eps, int32_t jt)
{
    auto A = [&](int i, int j) -> double& { return a[i * n + j]; };
    int nn = n - 1, its = 0;
    double t = 0.0;
    while (nn >= 0) {
        int l;
        for (l = nn; l >= 1; l--) {
            double s = std::fabs(A(l - 1, l - 1)) + std::fabs(A(l, l));
            if (s == 0.0)
                s = 1.0;
            if (std::fabs(A(l, l - 1)) <= eps * s) {
                A(l, l - 1) = 0.0;
                break;
            }
        }
        double x = A(nn, nn);
        if (l == nn) {
            u[nn] = x + t;
            v[nn] = 0.0;
            nn--;
            its = 0;
            continue;
        }
        double y = A(nn - 1, nn - 1), w = A(nn, nn - 1) * A(nn - 1, nn);
        if (l == nn - 1) {
            double p = 0.5 * (y - x), q = p * p + w, z = std::sqrt(std::fabs(q));
            x += t;
            if (q >= 0.0) {
                z = p + (p >= 0 ? std::fabs(z) : -std::fabs(z));
                u[nn - 1] = u[nn] = x + z;
                if (z != 0.0)
                    u[nn] = x - w / z;
                v[nn - 1] = v[nn] = 0.0;
            } else {
                u[nn - 1] = u[nn] = x + p;
                v[nn - 1] = -(v[nn] = z);
            }
            nn -= 2;
            its = 0;
            continue;
        }
        if (its >= jt)
            return -1;
        if (its == 10 || its == 20) {
            t += x;
            for (int i = 0; i <= nn; i++)
                A(i, i) -= x;
            double s = std::fabs(A(nn, nn - 1)) + std::fabs(A(nn - 1, nn - 2));
            y = x = 0.75 * s;
            w = -0.4375 * s * s;
        }
        ++its;
        int m;
        double p = 0, q = 0, r = 0, z;
        for (m = nn - 2; m >= l; m--) {
            z = A(m, m);
            r = x - z;
            double s = y - z;
            p = (r * s - w) / A(m + 1, m) + A(m, m + 1);
            q = A(m + 1, m + 1) - z - r - s;
            r = A(m + 2, m + 1);
            s = std::fabs(p) + std::fabs(q) + std::fabs(r);
            p /= s;
            q /= s;
            r /= s;
            if (m == l)
                break;
            double u1 = std::fabs(A(m, m - 1)) * (std::fabs(q) + std::fabs(r));
            double v1 = std::fabs(p) * (std::fabs(A(m - 1, m - 1)) + std::fabs(z) + std::fabs(A(m + 1, m + 1)));
            if (u1 <= eps * v1)
                break;
        }
        for (int i = m + 2; i <= nn; i++) {
            A(i, i - 2) = 0.0;
            if (i != m + 2)
                A(i, i - 3) = 0.0;
        }
        for (int k = m; k <= nn - 1; k++) {
            if (k != m) {
                p = A(k, k - 1);
                q = A(k + 1, k - 1);
                r = 0.0;
                if (k != nn - 1)
                    r = A(k + 2, k - 1);
                if ((x = std::fabs(p) + std::fabs(q) + std::fabs(r)) != 0.0) {
                    p /= x;
                    q /= x;
                    r /= x;
                }
            }
            double s = std::sqrt(p * p + q * q + r * r);
            if (p < 0)
                s = -s;
            if (s != 0.0) {
                if (k == m) {
                    if (l != m)
                        A(k, k - 1) = -A(k, k - 1);
                } else {
                    A(k, k - 1) = -s * x;
                }
                p += s;
                x = p / s;
                y = q / s;
                z = r / s;
                q /= p;
                r /= p;
                for (int j = k; j <= nn; j++) {
                    p = A(k, j) + q * A(k + 1, j);
                    if (k != nn - 1) {
                        p += r * A(k + 2, j);
                        A(k + 2, j) -= p * z;
                    }
                    A(k + 1, j) -= p * y;
                    A(k, j) -= p * x;
                }
                int mmin = nn < k + 3 ? nn : k + 3;
                for (int i = l; i <= mmin; i++) {
                    p = x * A(i, k) + y * A(i, k + 1);
                    if (k != nn - 1) {
                        p += z * A(i, k + 2);
                        A(i, k + 2) -= p * r;
                    }
                    A(i, k + 1) -= p * q;
                    A(i, k) -= p;
                }
            }
        }
    }
    return 1;
}

// Roots of a[0] + a[1] x + ... + a[n] x^n (companion matrix eigenvalues).
int32_t SIGNAL_DLL_CALL qrrt(int32_t n, double* a, double* xr, double* xi, double eps, int32_t jt)
{
    if (n <= 0)
        return -1;
    std::vector<double> q((size_t)n * n, 0.0);
    for (int j = 0; j < n; j++)
        q[j] = -a[n - 1 - j] / a[n];
    for (int i = 1; i < n; i++)
        q[(size_t)i * n + (i - 1)] = 1.0;
    return qrtt(n, q.data(), xr, xi, eps, jt);
}
