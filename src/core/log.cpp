#include "log.h"

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <mutex>

namespace vmopen {

bool g_log_enabled = false;

static std::mutex g_log_mutex;

void LogInit(bool enable)
{
    g_log_enabled = enable || std::getenv("VMOPEN_LOG") != nullptr;
}

void LogPrintf(const char* fmt, ...)
{
    static const auto t0 = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::lock_guard<std::mutex> lk(g_log_mutex);
    std::fprintf(stderr, "[vmopen %9.4f] ", t);
    va_list ap;
    va_start(ap, fmt);
    std::vfprintf(stderr, fmt, ap);
    va_end(ap);
}

} // namespace vmopen
