# ESP-SDR scanner on the CYD (ESP32-2432S028)

Standalone spectrum analyzer: the CYD captures raw I/Q from its own ESP32
radio and renders a live spectrum + waterfall. No PC, no host software, no
Wi-Fi/BLE involvement.

RF backend is the capture code from **ESP-SDR / ESPARGOS** (GPL-3.0), cut down
to a direct API (no serial burst protocol).

## Build and flash

```sh
cd sdr
../.venv/bin/pio run -e cyd-sdr -t upload
```

Serial log is 115200 baud on `/dev/ttyUSB0`. `pio device monitor` refuses a
piped stdin, so use a small pyserial script if you want to capture output.

## What you see

```
+--------------------------------------------------+
| 2442MHz 40M AGC  PK2447 +12dB 45fps    header 14px|
+--------------------------------------------------+
| spectrum, 320 px, 15..151                         |
+--------------------------------------------------+
| waterfall, 44 rows, 153..196                      |
+--------------------------------------------------+
| 40M | -5M | +5M | AGC | HOLD     buttons 198..239  |
+--------------------------------------------------+
```

- **rate** cycles 16 / 40 / 80 MS/s (span)
- **-5M / +5M** move the centre frequency (2300..2600 MHz)
- **GAIN** cycles hardware AGC and manual gain steps
- **HOLD** freezes the display
- tap the spectrum to retune to that frequency
- **long-press the header (top 14 px) for 1.5 s** to re-run touch calibration

## Hardware facts verified on this unit

| item | value |
| --- | --- |
| MCU | ESP32-D0WD-V3, 4 MB flash, CH340 @ 115200 |
| LCD | ST7789, 240x320 native, on HSPI |
| LCD pins | SCLK 14, MOSI 13, MISO 12, CS 15, DC 2, BL 21 |
| LCD init | `Panel_ST7789`, `invert=false`, `setRotation(1)` = 320x240 |
| touch | XPT2046, own bit-banged pins SCLK 25, MOSI 32, MISO 39, CS 33 |
| touch cal | NVS namespace `sdrtouch`, key `cal`, magic `CYD3` |

### Display: do not use `LGFX_AUTODETECT`

LovyanGFX caches the detected board in the NVS namespace `LovyanGFX`. This unit
had board 56 (`Guition_JC2432W328C`) cached, which selects `Panel_ILI9341_2`,
capacitive `Touch_CST816S` on 33/32/25 and backlight GPIO27. Result: black
screen. `src/display.cpp` therefore configures `Panel_ST7789` explicitly.

`setRotation(1)` or `3` gives landscape 320x240. `setRotation(6)` is portrait
(240x320) and crops the UI.

### Touch: Z must be read in differential mode

Pen detection is the part that bit us. The Z channels **must** be read with
`SER/DFR = 1` (command bytes `0xB1` / `0xC1`). With SER mode (`0xB0` / `0xC0`)
Z1 reads 0 and Z2 reads ~4095 on every sample, so pressure is never detected
and taps are ignored (or register only by luck).

Working command set:

```c
z1 = cmd(0xB1);  z2 = cmd(0xC1);  z = z1 + 4095 - z2;   /* press if z > 250 */
x  = cmd(0xD0);  y  = cmd(0x90);
```

Timing: 4 us per SCLK half-period, 10 us after CS goes low (125 kHz). Bit-bang
is used instead of the HSPI peripheral because the usable timing is easier to
guarantee; the display bus is independent.

### Calibration maths

Four corner taps are fitted with least squares. Each screen axis is fitted
against **both** raw channels, then the pairing with the smaller total error
wins:

- pair A: `sx <- rx`, `sy <- ry`
- pair B: `sx <- ry`, `sy <- rx`

On this board pair B wins with an error of 44 vs 118374:

```
sx = 0.0900 * rawY - 23.5
sy = 0.0737 * rawX - 28.6
```

Deriving both axes from one raw channel is the classic bug in ported CYD touch
code — it leaves the UI alive but every tap in the wrong place.

Calibration is only saved when all four taps succeed. A timeout falls back to a
rough linear map and leaves the blob unsaved so the next boot retries. The
calibration screen waits indefinitely and shows live `x/y/z1/z2/z` values;
hold **BOOT (GPIO0)** to skip.

## Capture notes

- snapshot only; 80 / 40 / 16 MS/s (dump clock codes 1 / 2 / 0)
- up to 16,380 I/Q samples; `STATUS & 0x7fff` is the sample count
- I/Q word: I bits 0..9, Q bits 10..19, both signed; `words[0] >> 20` = gain
- `0x3ffe8000-0x3fff8000` must stay reserved (`SOC_RESERVE_MEMORY_REGION`) or
  the dump engine corrupts the heap; `src/sram_guard.ld` asserts this
- 1024-point float radix-2 FFT with precomputed twiddles, max-pooled to 320
  columns, fftshifted
- 60 dB adaptive display window: jumps up immediately, drifts down 5 %/frame
- ~44-47 fps, 0 failed captures, ~136 KB free heap
