/* vmping: minimal handshake test for Vimu MSO devices.
 * usage: vmping [hex-cmd ...]   e.g. vmping 02ff0b0000000000
 * Performs the serial reset + key exchange + identify, then sends each
 * given command and prints the reply. */
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static libusb_device_handle* h;

static int xfer(unsigned char ep, unsigned char* b, int n, int to)
{
    int tr = 0;
    int r = libusb_bulk_transfer(h, ep, b, n, &tr, to);
    return r < 0 ? r : tr;
}

static int cmd(const unsigned char* out, int n, unsigned char* in)
{
    int r = xfer(0x01, (unsigned char*)out, n, 3000);
    if (r < 0) {
        printf("OUT failed %s\n", libusb_error_name(r));
        return r;
    }
    r = xfer(0x81, in, 1024, 3000);
    printf(">");
    for (int i = 0; i < n; i++)
        printf(" %02x", out[i]);
    printf("\n<");
    if (r < 0)
        printf(" %s", libusb_error_name(r));
    for (int i = 0; i < r && i < 32; i++)
        printf(" %02x", in[i]);
    printf("\n");
    return r;
}

int main(int argc, char** argv)
{
    unsigned char in[1024];
    libusb_init(NULL);
    h = libusb_open_device_with_vid_pid(NULL, 0xd4a2, 0x5662);
    if (!h)
        h = libusb_open_device_with_vid_pid(NULL, 0xd4a2, 0x5663);
    if (!h) {
        printf("no device\n");
        return 1;
    }
    int cfg = -1;
    if (getenv("FORCE_SETCFG") || libusb_get_configuration(h, &cfg) < 0 || cfg != 1)
        libusb_set_configuration(h, 1);
    libusb_claim_interface(h, 0);
    static const unsigned char reset[10] = {0xff, 0xff, 0xff, 0xff, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55};
    int ok = cmd(reset, 10, in) == 10 && !memcmp(in, reset, 10);
    if (ok) {
        unsigned char k[10] = {0x67, 0xc6, 0x69, 0x73, 0x51};
        ok = cmd(k, 5, in) == 5;
        memcpy(k + 5, in, 5);
        ok = ok && cmd(k, 10, in) == 4 && in[0] == 0xaa;
    }
    printf("handshake %s\n", ok ? "OK" : "FAILED");
    if (ok) {
        static const unsigned char ident[8] = {0, 0, 0xfd, 0, 0, 0, 0, 0};
        cmd(ident, 8, in);
        for (int a = 1; a < argc; a++) {
            unsigned char b[64];
            int n = 0;
            for (const char* p = argv[a]; p[0] && p[1] && n < 64; p += 2) {
                char t[3] = {p[0], p[1], 0};
                b[n++] = (unsigned char)strtoul(t, NULL, 16);
            }
            cmd(b, n, in);
        }
    }
    libusb_release_interface(h, 0);
    libusb_close(h);
    libusb_exit(NULL);
    return ok ? 0 : 2;
}
