# cyd_esp_sdr

A standalone **spectrum analyzer for the Cheap Yellow Display** (ESP32-2432S028).
The CYD tunes its own ESP32 radio, captures raw I/Q snapshots, runs a 1024-point
FFT on-chip and draws a live spectrum plus waterfall. No PC, no host software,
no Wi-Fi or BLE — the whole instrument is the $15 board on your desk.

```
┌──────────────────────────────────────────────────┐
│ 2442MHz 40M AGC  PK2447 +12dB 45fps              │
├──────────────────────────────────────────────────┤
│ spectrum                                         │
├──────────────────────────────────────────────────┤
│ waterfall                                        │
├──────────────────────────────────────────────────┤
│  40M  │  -5M  │  +5M  │  AGC  │  HOLD            │
└──────────────────────────────────────────────────┘
```

## Features

- Live 320-column spectrum and scrolling waterfall, ~45 fps
- 16 / 40 / 80 MS/s raw I/Q capture
- 2300–2600 MHz tuning (ESP32 2.4 GHz radio), tap the spectrum to retune
- Hardware AGC or manual gain steps, HOLD to freeze the display
- Peak marker with frequency and relative dB
- Touch UI with persistent four-corner calibration
- Self-contained ESP-IDF firmware, single `factory` partition

## Hardware

Any ESP32-2432S028 "Cheap Yellow Display": ESP32-D0WD-V3, 4 MB flash,
240×320 ST7789 panel and an XPT2046 resistive touch controller on the usual
CYD pins (see [NOTES.md](NOTES.md) for the full table).

The firmware writes the radio's raw sample aperture directly, so it is specific
to the **classic ESP32**; it will not work on S2/S3/C3 parts.

## Build and flash

Requires [PlatformIO](https://platformio.org/):

```sh
pio run -e cyd-sdr -t upload
```

If auto-detection grabs the wrong serial port:

```sh
pio run -e cyd-sdr -t upload --upload-port /dev/ttyUSB0
```

Dependencies (ESP-IDF 5.3 and LovyanGFX 1.3.0) are fetched automatically.
The build asserts at link time that static data does not overlap the memory
aperture used by the capture engine; if you see the `sram_guard.ld` error, the
firmware grew into the reserved region.

## Controls

| control | action |
| --- | --- |
| **40M** | cycle capture clock: 16 / 40 / 80 MS/s |
| **-5M** / **+5M** | move the centre frequency by ±5 MHz |
| **AGC** | cycle hardware AGC and manual gain steps |
| **HOLD** | freeze spectrum and waterfall |
| tap spectrum | retune to the tapped frequency |
| long-press header 1.5 s | re-run touch calibration |

On first boot the firmware runs a four-tap touch calibration. It waits as long
as necessary and shows live raw touch values; hold **BOOT** (GPIO0) to skip it
and use the fallback mapping.

## How it works

1. `sdr_rx.c` initialises the PHY in receive-only test mode and arms the packet
   dump engine to write I/Q samples into a fixed 64 KiB aperture at
   `0x3ffe8000`. This is the same trick ESPARGOS/ESP-SDR uses; the serial burst
   protocol and command shell are stripped and replaced by a direct C API.
2. Each capture returns up to 16,380 interleaved I/Q pairs (10-bit signed each,
   packed into 20-bit words). The `0x3ffe8000–0x3fff8000` region is reserved
   from the heap or the dump engine corrupts it.
3. `main.cpp` windows the samples (Blackman-Harris), runs a float radix-2 FFT
   with precomputed twiddles, fftshifts and max-pools to 320 columns.
4. A 60 dB adaptive window drives the colour ramp (jumps up instantly, drifts
   down 5 %/frame) and the waterfall ring buffer.

Measured on a stock CYD: 44–47 fps, no failed captures, ~136 KB free heap.

## Credits and licence

- RF backend derived from **ESP-SDR / ESPARGOS** by the ESP-SDR project
  (`main/targets/esp32/receiver.c`), **GPL-3.0**.
- Display driver: [LovyanGFX](https://github.com/lovyan03/LovyanGFX) 1.3.0 (BSD).
- Touch: original bit-bang XPT2046 driver, documented in [NOTES.md](NOTES.md).

Because it contains GPL-3.0 code, this project is released under the
**GNU General Public License v3.0** — see [LICENSE](LICENSE).

## Legal

Receiving is passive: the firmware never transmits, injects or decodes any
protocol. You are still responsible for complying with the radio regulations
where you operate it. The 2.4 GHz ISM band rules vary by country.
