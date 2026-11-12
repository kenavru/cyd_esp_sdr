/* Public interface of the vendored ESP-SDR ESP32 receive backend.
 * Derived from ESPARGOS/esp-sdr main/targets/esp32/receiver.c (GPL-3.0).
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Hardware capture clock, from the dump engine. Not software decimation. */
typedef enum {
    SDR_CLOCK_16M = 0,
    SDR_CLOCK_80M = 1,
    SDR_CLOCK_40M = 2,
} sdr_clock_t;

#define SDR_MAX_SAMPLES 16380u

/* Bring up Wi-Fi/PHY in receive-only mode and prepare the RF front end. */
void sdr_init(void);

/* Retune and recalibrate. Returns false if the request is out of range. */
bool sdr_tune_mhz(unsigned mhz);
unsigned sdr_frequency_mhz(void);

/* Capture `n` raw I/Q words (I in bits 0..9, Q in 10..19, signed).
 * Power the radio up once, tune, then call this in a loop. */
bool sdr_acquire(unsigned n, sdr_clock_t clock, unsigned *elapsed_us);

/* The captured words live at the reserved SRAM aperture; valid until the
 * next sdr_acquire(). */
const uint32_t *sdr_iq(void);

unsigned sdr_gain_max(void);
void sdr_gain_hardware(void);
void sdr_gain_manual(unsigned code);
bool sdr_gain_is_hardware(void);

/* Front-end filter: -1 selects the automatic value, 0..127 a raw DAC code. */
void sdr_set_filter(int dcap);
int sdr_get_filter(void);

#ifdef __cplusplus
}
#endif
