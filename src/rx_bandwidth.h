#pragma once
#include <stdint.h>

/* Original ESP32: seven-bit BBTOP 1/2, 2472 MHz, gain 72, IQ10.
 * Code 0 is wider than the measured span. Numeric max uses code 8. */
#define RX_BANDWIDTH_MIN 12u
#define RX_BANDWIDTH_MAX 67u

typedef struct {
    uint8_t dcap, mhz;
} rx_bandwidth_point_t;

static inline uint8_t rx_bandwidth_dcap(unsigned mhz) {
    static const rx_bandwidth_point_t cal[] = {
        {8, 67},  {12, 55}, {16, 48}, {24, 38},  {32, 32}, {48, 25},
        {64, 20}, {80, 17}, {96, 15}, {112, 14}, {127, 12}};
    const rx_bandwidth_point_t *curve = cal;
    unsigned count = sizeof(cal) / sizeof(cal[0]);
    if (!mhz) return 0;
    if (mhz >= curve[0].mhz) return curve[0].dcap;
    if (mhz <= curve[count - 1].mhz) return curve[count - 1].dcap;
    for (unsigned i = 1; i < count; i++)
        if (mhz >= curve[i].mhz) {
            unsigned span = curve[i - 1].mhz - curve[i].mhz;
            return curve[i - 1].dcap +
                   ((curve[i].dcap - curve[i - 1].dcap) * (curve[i - 1].mhz - mhz) + span / 2) / span;
        }
    return curve[count - 1].dcap;
}
