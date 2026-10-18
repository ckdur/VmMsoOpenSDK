// Times CalFreq on 1M samples (noise and a sine), prints the results so
// vendor and open implementations can be compared for speed and output.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
#include "VmMultMsoLib.h"

int main()
{
    const unsigned n = 1048576;
    std::vector<double> noise(n), sine(n);
    unsigned s = 12345;
    for (unsigned i = 0; i < n; i++) {
        s = s * 1103515245u + 12345u;
        noise[i] = ((s >> 16) % 3) * 0.0538 - 0.05;
        sine[i] = 1.2 * std::sin(2 * M_PI * 1000.0 * i / 20e6) + ((s >> 16) % 3) * 0.0538;
    }
    for (auto* b : {&noise, &sine}) {
        auto t0 = std::chrono::steady_clock::now();
        int r = 0;
        for (int k = 0; k < 5; k++)
            r = CalFreq(b->data(), n, 0.053825, 20000000);
        auto t1 = std::chrono::steady_clock::now();
        printf("CalFreq=%d freq=%.6f phase=%.4f +duty=%.4f -duty=%.4f  %.1f ms/call\n", r, GetFreq(), GetPhase(),
               GetPositiveDuty(), GetNegativeDuty(),
               std::chrono::duration<double, std::milli>(t1 - t0).count() / 5);
    }
    return 0;
}
