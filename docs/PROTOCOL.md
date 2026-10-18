# Vimu MSO USB protocol

Reverse engineered from `libvmmso.so` / `libvmmultmso.so` 1.22 and checked
against USB traces of an MSO20BL (hardware 2.0, protocol 12). The
implementation is in `src/core`.

## Transport

| Endpoint | Use |
|---------|-----|
| `0x01` bulk OUT | requests |
| `0x81` bulk IN  | replies (read with a 1024 byte buffer) |
| `0x82` bulk IN  | capture data (read in chunks of up to 4 MiB) |
| `0x06` bulk OUT | DDS waveform upload |

USB IDs `d4a2:5662` and `d4a2:5663`, configuration 1, interface 0. Every
transfer uses a 3000 ms timeout.

**Do not issue `SET_CONFIGURATION` when configuration 1 is already active.**
The request resets the host data toggles but the firmware keeps its own, so
after a session that left an odd number of packets on an endpoint the device
stops answering until it is reset or re-plugged. (The vendor library has this
bug: a second program run after one ending with an odd packet count hangs in
the handshake.) The open library only selects the configuration when needed and
recovers a stuck device with a port reset.

## Request format

```
[module u8][channel u8][cmd u8][value u32][payload ...]
```

* Multi byte fields are big endian when the protocol version reported by the
  device is greater than 9 (all current devices).
* Without payload a single `0x00` byte is appended (8 byte request).
* Modules: `0x00` system, `0x01` DDS, `0x02` oscilloscope, `0x10` IO.
* Channel `0xff` addresses the whole module, otherwise channel numbers start at 1.

Replies start with `[module][channel][cmd][status]`, status `1` = ok.

A single worker thread executes the request queue in order:
optional pre-delay, write the request, optional post-delay, then either read a
reply, read bulk data (`0x82`), or write bulk data (`0x06`) followed by a reply.

## Handshake

| Host sends | Device answers |
|-----------|----------------|
| `ff ff ff ff ff 11 22 33 44 55` | same 10 bytes |
| 5 random bytes | 5 random bytes |
| host 5 bytes + device 5 bytes | `aa 55 55 aa` |

Then, all on module 0 / channel 0:

| cmd | reply contents |
|-----|----------------|
| `fd` identify | `[4]` protocol, `[5..0x14]` model name, `[0x15]` hw version, `[0x16]` sub version, `[0x17]` RAM index, `[0x18..]` unique id (BE u32) |
| `fc` | ADC calibration |
| `fb` | input stage calibration |
| `fa 0` | calibration table length: u16 total, u16 chunk, u16 count; then `fa 1..count` return the chunks |
| `f0` / `f1` | DDS DAC level / DDS zoom calibration |
| `f2` | IO DAC calibration |
| `f4` | power supply calibration |

The handshake is retried every 200 ms for 20 ticks and restarted once on
failure. The capability descriptors (channels, sample rates, ranges, DDS
waveforms, ...) are not read from the device: they are static tables selected
by model name and hardware version (`tools/gen_devtables.py` extracts them from
the vendor library into `src/core/devtables.cpp`).

Watchdog: `00 00 fe` with value 1 and a u32 timeout (6000 ms) enables it,
value 2 is the keep-alive sent every 1200 ms.

## Oscilloscope (module 2)

| cmd | meaning |
|-----|---------|
| `ff 12` | reset |
| `ch 17` | ADC adjust, payload u32 a, u32 b |
| `ch 01` / `ch 10` | start / stop channel |
| `ff 0a` / `ff 0b` | start (value = mask) / stop logic channels |
| `ff 15` | stream mode (value 0/1) |
| `ff 08` | sample rate index (payload u8) |
| `ch 06` | input attenuator (value = index, payload u16 calibration) |
| `ch 04` | gain (value = index, payload u16 DAC code, u16 calibration[, u16 model constant]) |
| `ch 19` | AC/DC coupling (payload u8, 1 = AC) |
| `ff 11` | trigger mode |
| `ch 18` | trigger style (`1→3, 2→4, 4→5, 8→6, 0x10→7, 0x20→8, 0x40→9, 0x80→10, 0x100→11`); `20`/`21` for digital/external sources |
| `ch 13` / `ch 24` | trigger level / sensitivity, payload u16 level ADC, u16 sensitivity ADC |
| `ff 27` | pulse width trigger, payload u32 down ns, u32 up ns |
| `ff 02` | capture: value = samples, payload u32 pre-trigger samples, u32 time at max rate[, u32, u32] |
| `ff 03` | read data: value = bytes / 2, then `bytes` on `0x82` |
| `ff 09` | abort |
| `ff 16` | poll capture state |

The capture reply carries status 1 plus the u32 trigger position; status
`0x10`/`0x11` means "not ready", then the host polls with `ff 16`.
Gain DAC code = `(dac + 1250) * 0.5 * 4096 / vref` with vref 3300 mV
(4250 MSO21 V1, 2500 MSO41).

Data layout: one block per sample with the captured analog channels in order,
then the logic bits. 8 bit models use bytes, 12 bit models (MSO21 family,
MSO41) 16 bit little endian words. Voltage = `adc * para * zoom + bias`;
the first 6 and last 4 samples of a capture are replaced by their neighbours.

Stream mode: `ff 3f 1`, `ff 15 1`, then repeated `ff 02` with value = block
size and payload `[block / 2, block]`; reply status 2 means a block is ready.

## DDS (module 1)

The host synthesises the waveform (8192 DAC codes for the MSO20) and uploads
it with `ch 49` (value = bytes) followed by the bulk write on `0x06`.
Frequency is set by `ch 42`, payload u64 phase increment
`(2^64 - 1) / clock * f`. Other commands: `50` output mode, `51`/`52`/`55`
trigger, `54` gate, `56`/`43`/`42` sweep, `57` burst, `58` enable,
`01`/`10` start/stop, `ff 61` synchronise.

## IO (module 0x10)

`ff 01`/`ff 10` start/stop, `io 31` output state, `ff 30` direction mask,
`io 34` pulse (payload `1e8/hz - 1`, `duty * 1e8 / 100 / hz`), `ff 35` pulse
sync, `ff 32` read inputs (reply byte 4 = state), `d+1 36` DAC value.
