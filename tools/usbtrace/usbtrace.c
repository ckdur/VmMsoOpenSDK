/*
 * usbtrace - LD_PRELOAD shim that logs libusb synchronous traffic.
 *
 * Used to observe the wire protocol of a Vimu MSO device while it is driven by
 * any libusb based program (vendor library or this open implementation).
 *
 *   USBTRACE_FILE=trace.txt LD_PRELOAD=./libusbtrace.so ./program
 *
 * USBTRACE_MAX limits the number of payload bytes dumped per transfer
 * (default 64, -1 for all).
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include <libusb.h>

static FILE *out;
static int max_dump = 64;
static pthread_mutex_t lk = PTHREAD_MUTEX_INITIALIZER;
static struct timespec t0;

static void init(void)
{
    if (out)
        return;
    const char *f = getenv("USBTRACE_FILE");
    out = f ? fopen(f, "w") : stderr;
    if (!out)
        out = stderr;
    const char *m = getenv("USBTRACE_MAX");
    if (m)
        max_dump = atoi(m);
    clock_gettime(CLOCK_MONOTONIC, &t0);
}

static double now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) / 1e9;
}

static void dump(const char *tag, int ep, const unsigned char *d, int req, int act, int rc, unsigned to)
{
    pthread_mutex_lock(&lk);
    init();
    fprintf(out, "%10.6f T%ld %s ep=%02x req=%d act=%d rc=%d to=%u :", now(),
            (long)syscall(SYS_gettid), tag, ep, req, act, rc, to);
    int n = act;
    if (max_dump >= 0 && n > max_dump)
        n = max_dump;
    for (int i = 0; i < n; i++)
        fprintf(out, " %02x", d[i]);
    if (n < act)
        fprintf(out, " ...");
    fprintf(out, "\n");
    fflush(out);
    pthread_mutex_unlock(&lk);
}

int libusb_bulk_transfer(libusb_device_handle *h, unsigned char ep, unsigned char *data,
                         int length, int *transferred, unsigned int timeout)
{
    static int (*real)(libusb_device_handle *, unsigned char, unsigned char *, int, int *, unsigned int);
    if (!real)
        real = dlsym(RTLD_NEXT, "libusb_bulk_transfer");
    int tr = 0;
    if (!(ep & 0x80))
        dump("OUT", ep, data, length, length, 0, timeout);
    int rc = real(h, ep, data, length, &tr, timeout);
    if (transferred)
        *transferred = tr;
    if (ep & 0x80)
        dump("IN ", ep, data, length, tr, rc, timeout);
    else if (rc || tr != length)
        dump("OUT-RES", ep, data, length, tr, rc, timeout);
    return rc;
}

int libusb_control_transfer(libusb_device_handle *h, uint8_t rt, uint8_t rq, uint16_t val,
                            uint16_t idx, unsigned char *data, uint16_t len, unsigned int to)
{
    static int (*real)(libusb_device_handle *, uint8_t, uint8_t, uint16_t, uint16_t, unsigned char *, uint16_t, unsigned int);
    if (!real)
        real = dlsym(RTLD_NEXT, "libusb_control_transfer");
    int rc = real(h, rt, rq, val, idx, data, len, to);
    pthread_mutex_lock(&lk);
    init();
    fprintf(out, "%10.6f CTRL rt=%02x rq=%02x val=%04x idx=%04x len=%u rc=%d\n", now(), rt, rq, val, idx, len, rc);
    pthread_mutex_unlock(&lk);
    return rc;
}

#define SIMPLE(name, proto, args, fmt, ...)                         \
    int name proto                                                  \
    {                                                               \
        static int(*real) proto;                                    \
        if (!real)                                                  \
            real = dlsym(RTLD_NEXT, #name);                         \
        int rc = real args;                                         \
        pthread_mutex_lock(&lk);                                    \
        init();                                                     \
        fprintf(out, "%10.6f " #name " " fmt " rc=%d\n", now(), __VA_ARGS__, rc); \
        fflush(out);                                                \
        pthread_mutex_unlock(&lk);                                  \
        return rc;                                                  \
    }

SIMPLE(libusb_claim_interface, (libusb_device_handle * h, int i), (h, i), "if=%d", i)
SIMPLE(libusb_release_interface, (libusb_device_handle * h, int i), (h, i), "if=%d", i)
SIMPLE(libusb_set_configuration, (libusb_device_handle * h, int c), (h, c), "cfg=%d", c)
SIMPLE(libusb_open, (libusb_device * d, libusb_device_handle **h), (d, h), "dev=%p", (void *)d)
