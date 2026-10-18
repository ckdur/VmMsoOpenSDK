// Times the GUI's FFT path (window_i + real_fft_signal + Fft_Amplitude).
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
#include "vmfft.h"
#include "vmreal_fft.h"
#include "vmwindow.h"

int main()
{
    for (unsigned n : {65536u, 1048576u}) {
        std::vector<double> re(n), im(n), mag(n);
        auto t0 = std::chrono::steady_clock::now();
        const int reps = 5;
        for (int k = 0; k < reps; k++) {
            for (unsigned i = 0; i < n; i++) {
                re[i] = std::sin(i * 0.01) + (i % 7) * 0.01;
                im[i] = 0;
            }
            window_i(HANNING, re.data(), n, 0, 0);
            real_fft_signal(re.data(), im.data(), n, n, 8);
            Fft_Amplitude(re.data(), im.data(), mag.data(), n, 1);
        }
        auto t1 = std::chrono::steady_clock::now();
        printf("n=%u  %.1f ms  mag[100]=%.9g mag[n/3]=%.9g\n", n,
               std::chrono::duration<double, std::milli>(t1 - t0).count() / reps, mag[100], mag[n / 3]);
    }
    return 0;
}
