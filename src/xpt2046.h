#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Raw XPT2046 sample. Bits are the 12-bit converter results. */
typedef struct {
    uint16_t x, y;
    uint16_t z1, z2;
    int32_t z;
} xpt_raw_t;

/* Affine per-axis fit: screen = a * raw + b. The raw channel feeding each
 * screen axis can be swapped on some glass. */
typedef struct {
    float sx_a, sx_b;
    float sy_a, sy_b;
    bool swapped_x, swapped_y;
    bool valid;
} xpt_cal_t;

void xpt_init(void);

/* One low-level sample sequence (Y, Z1, Z2, X). Returns false when the
 * controller reports no pressure. */
bool xpt_read(xpt_raw_t *out);

/* Averaged raw reading while pressed. Returns false if not pressed. */
bool xpt_touch_raw(uint16_t *rx, uint16_t *ry);

/* Calibration persistence (NVS namespace "touch"). */
bool xpt_cal_load(xpt_cal_t *cal);
void xpt_cal_save(const xpt_cal_t *cal);
void xpt_cal_apply(const xpt_cal_t *cal);

/* Map a raw sample with the active calibration. */
void xpt_map(uint16_t rx, uint16_t ry, int *sx, int *sy);

#ifdef __cplusplus
}
#endif
