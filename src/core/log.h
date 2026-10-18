// Minimal logging. Enabled by InitDll(en_log=1) or the VMOPEN_LOG environment variable.
#pragma once

namespace vmopen {

extern bool g_log_enabled;
void LogPrintf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void LogInit(bool enable);

} // namespace vmopen

#define LOG(...)                               \
    do {                                       \
        if (::vmopen::g_log_enabled)           \
            ::vmopen::LogPrintf(__VA_ARGS__);  \
    } while (0)
