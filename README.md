# VmMsoOpenSDK

Open-source replacement for the closed libraries of the
[Vimu MSO SDK](https://github.com/vimu-tech/VmMsoSDK) (VmMsoSDK 1.22) for the Vimu / Instrustar
USB oscilloscopes (MSO10, MSO20, MSO21, MSO41 families):

| Library | Header | Vendor binary it replaces |
|---------|--------|---------------------------|
| `libvmmso.so.1.22` | `include/VmMsoLib.h` | single device API |
| `libvmmultmso.so.1.22` | `include/VmMultMsoLib.h` | multi device API (incl. stream mode) |
| `libvmsignal.so.1.0` | `include/vmsignal/inc/...` | DSP library used by the GUI |

The libraries keep the vendor SONAMEs and C/C++ ABI, so existing programs
(including the vendor builds of VimuLink) run on them unchanged. The
`VimuLink` Qt GUI from the SDK is included in `gui/VimuLink` and is built
against the open libraries.

The implementation was written from a reverse engineering of the vendor
binaries (Ghidra) and USB traces; the protocol is documented in
[docs/PROTOCOL.md](docs/PROTOCOL.md).

## Building

Requirements: CMake ≥ 3.16, a C++17 compiler, libusb-1.0, and Qt 6 (or 5)
with Quick/QuickControls2 for the GUI.

```sh
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev qt6-declarative-dev
cmake -B build -S .
cmake --build build -j
sudo cp udev/99-vmmso.rules /etc/udev/rules.d/ && sudo udevadm control --reload
```

Options: `-DVMOPEN_BUILD_GUI=OFF`, `-DVMOPEN_BUILD_TESTS=OFF`.

Run the GUI: `build/gui/VimuLink/VimuLink`.
Use the libraries with an existing vendor build: put `build/` first in
`LD_LIBRARY_PATH`.

Set `VMOPEN_LOG=1` to get a log of the device communication on stderr.

### macOS

```sh
brew install cmake pkg-config libusb qt
cmake -B build -S .
cmake --build build -j
open build/gui/VimuLink/VimuLink.app
```

No udev rule or driver is needed: macOS lets user programs claim the device
through libusb. The libraries are built as `libvmmso.1.22.dylib`,
`libvmmultmso.1.22.dylib` and `libvmsignal.1.0.dylib` (`DYLD_LIBRARY_PATH`
instead of `LD_LIBRARY_PATH`), and the GUI as an app bundle that saves its
recordings in `~/Library/Application Support/VimuLink/datas`. Run
`macdeployqt` on the bundle to make it self contained. `tools/usbtrace` and
`tools/usbreset` are Linux only.

## Status

Verified on an MSO20BL (hardware 2.0) by running the same programs on the
vendor and the open libraries and comparing results and USB traces:

* handshake, identification, calibration tables, all capability queries
* oscilloscope: ranges, sample rates, trigger (auto/normal, styles, level,
  pre-trigger), AC/DC, captures, abort, voltage/ADC conversion, trigger point
  refinement, frequency / duty / phase measurement
* stream mode
* DDS: all waveforms, sweep, burst, triggers
* IO lines, pulse output, input reading, DAC
* VimuLink: real time and stream modes run at the same frame rate as with the
  vendor libraries
* libvmsignal: windows, FFT, FIR/IIR design, freqz, filtering match the vendor
  to ≤ 1e-6 (`tests/sigtest.cpp`, `tests/sigcompare.py`)

The per model differences of the other oscilloscopes (MSO10, MSO20 V4,
MSO21 family, MSO41: 12 bit sample words, channel/logic packing, gain DAC
constants, ...) are implemented from the vendor code but untested on hardware.
Multimeter and power supply modules (not present on the MSO20) are stubs.
libvmsignal implements everything VimuLink uses; some rarely used exports of
the vendor library (correlation, Kaiser order estimation, `fir1`, the
`*_core` classes) are not provided.

Differences from the vendor behaviour, all deliberate:

* the device is not left unresponsive between sessions (see PROTOCOL.md)
* each queued DDS upload uses the buffer computed for that call (the vendor
  library may send a later buffer for earlier calls)
* stop commands are flushed before closing or switching capture mode
* `GetDACmV` returns the last value set (the vendor always returns 0)

## Tests and tools

| | |
|-|-|
| `tests/probe_single.cpp` | queries every property and captures through `VmMsoLib.h` |
| `tests/probe_multi.cpp` | DDS, IO, DAC, normal trigger, stream mode through `VmMultMsoLib.h` |
| `tests/sigtest.cpp` + `sigcompare.py` | libvmsignal differential test |
| `tests/bench_*.cpp` | speed comparisons |
| `tools/usbtrace` | `LD_PRELOAD` libusb tracer (`USBTRACE_FILE`, `USBTRACE_MAX`) |
| `tools/vmping` | minimal handshake tester |
| `tools/trace_dds.py` | compares DDS uploads of two traces |
| `tools/gen_devtables.py` | extracts the capability tables from the vendor library |
| `traces/` | reference outputs and USB traces of the vendor library |

The probes build against either implementation, e.g. for the vendor library:
`g++ -Iinclude tests/probe_multi.cpp /path/to/libvmmultmso.so.1.22`.
For headless GUI tests: `QT_QPA_PLATFORM=offscreen VIMULINK_AUTOCAPTURE=1`
(or `=stream`) starts capturing as soon as a device is found.

## License

MIT, see [LICENSE](LICENSE). The public headers, the vmsignal headers and the
VimuLink GUI are taken from the vendor SDK, which is also MIT licensed.
