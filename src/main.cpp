/* Standalone ESP-SDR scanner for the ESP32-2432S028 (CYD).
 *
 * Raw I/Q comes from the undocumented ESP32 receive-dump engine (the ESP-SDR
 * backend in sdr_rx.c). This file does the on-device work that the original
 * project did on a PC: deinterleave IQ10, window, FFT, dB scaling, spectrum
 * and waterfall rendering, plus a small touch UI.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "display.hpp"
#include "sdr_rx.h"
#include "xpt2046.h"

/* ---------------- layout ---------------- */
#define SCREEN_W 320
#define SCREEN_H 240
#define HEADER_H 14
#define SPEC_Y   15
#define SPEC_H   137
#define WF_Y     (SPEC_Y + SPEC_H + 1) /* 153 */
#define WF_H     44
#define BTN_Y    198
#define BTN_H    42
#define BTN_W    (SCREEN_W / 5)

/* ---------------- FFT ---------------- */
#define FFT_N 1024
#define COLS  SCREEN_W

static float *g_re, *g_im, *g_win, *g_tw_cos, *g_tw_sin;
static float g_col_db[COLS];
static float g_top_db = 108.0f;
static float g_bottom_db = 48.0f;
static bool g_scale_ready = false;

static uint16_t *g_wf;        /* WF_H rows x COLS */
static uint16_t g_prev_h[COLS];
static uint16_t g_wf_row[COLS];

/* ---------------- state ---------------- */
static const unsigned g_rates[3] = {16, 40, 80};
static const sdr_clock_t g_clocks[3] = {SDR_CLOCK_16M, SDR_CLOCK_40M, SDR_CLOCK_80M};
static const int g_gains[6] = {0, 20, 32, 44, 56, 68};

static int g_rate_idx = 1;        /* 40 MS/s default */
static unsigned g_center_mhz = 2442;
static bool g_tuned = false;
static bool g_hold = false;
static int g_gain_idx = 0;        /* 0 = hardware AGC */
static float g_peak_db = 0;
static unsigned g_peak_mhz = 0;
static float g_fps = 0;
static int g_err_count = 0;
static uint32_t g_frames = 0, g_fail_total = 0;
static char g_status[16] = "START";

static xpt_cal_t g_cal;

/* ================================================================ */
/* colour                                                           */
/* ================================================================ */
static uint16_t heat(float v) {
    if (v < 0) v = 0;
    if (v > 1) v = 1;
    static const uint8_t s[6][3] = {
        {0, 0, 0}, {0, 0, 210}, {0, 180, 255}, {0, 255, 80}, {255, 220, 0}, {255, 255, 255}};
    float f = v * 5.0f;
    int i = (int)f;
    if (i > 4) i = 4;
    float t = f - (float)i;
    int r = (int)(s[i][0] + (s[i + 1][0] - s[i][0]) * t);
    int g = (int)(s[i][1] + (s[i + 1][1] - s[i][1]) * t);
    int b = (int)(s[i][2] + (s[i + 1][2] - s[i][2]) * t);
    return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

/* ================================================================ */
/* FFT                                                              */
/* ================================================================ */
static void fft_tables(void) {
    const float pi2 = 6.283185307179586f;
    for (int k = 0; k < FFT_N / 2; k++) {
        g_tw_cos[k] = cosf(-pi2 * (float)k / FFT_N);
        g_tw_sin[k] = sinf(-pi2 * (float)k / FFT_N);
    }
    for (int j = 0; j < FFT_N; j++) {
        g_win[j] = 0.5f * (1.0f - cosf(pi2 * (float)j / FFT_N));
    }
}

static void fft_run(void) {
    float *re = g_re, *im = g_im;
    /* natural order in, bit-reversed permutation first */
    for (int i = 1, j = 0; i < FFT_N; i++) {
        int bit = FFT_N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float t = re[i];
            re[i] = re[j];
            re[j] = t;
            t = im[i];
            im[i] = im[j];
            im[j] = t;
        }
    }
    for (int len = 2; len <= FFT_N; len <<= 1) {
        int half = len >> 1;
        int step = FFT_N / len;
        for (int base = 0; base < FFT_N; base += len) {
            for (int j = 0; j < half; j++) {
                int k = j * step;
                float wr = g_tw_cos[k], wi = g_tw_sin[k];
                int a = base + j, b = a + half;
                float tr = re[b] * wr - im[b] * wi;
                float ti = re[b] * wi + im[b] * wr;
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
    }
}

/* Pull one snapshot, window it, FFT it and fold the bins into screen columns.
 * Column 0 is the lowest frequency (-fs/2). */
static bool process_frame(void) {
    unsigned us = 0;
    if (!sdr_acquire(FFT_N, g_clocks[g_rate_idx], &us)) return false;

    const uint32_t *w = sdr_iq();
    float mi = 0, mq = 0;
    for (int j = 0; j < FFT_N; j++) {
        uint32_t v = w[j];
        mi += (float)(((int32_t)(v << 22)) >> 22);
        mq += (float)(((int32_t)(v << 12)) >> 22);
    }
    mi /= FFT_N;
    mq /= FFT_N;

    for (int j = 0; j < FFT_N; j++) {
        uint32_t v = w[j];
        float i = (float)(((int32_t)(v << 22)) >> 22) - mi;
        float q = (float)(((int32_t)(v << 12)) >> 22) - mq;
        g_re[j] = i * g_win[j];
        g_im[j] = q * g_win[j];
    }
    fft_run();

    const int half = FFT_N / 2;
    float frame_max = -1e9f;
    int peak_col = 0;
    for (int c = 0; c < COLS; c++) {
        int b0 = (int)((long long)c * FFT_N / COLS);
        int b1 = (int)((long long)(c + 1) * FFT_N / COLS);
        if (b1 <= b0) b1 = b0 + 1;
        float mx = 0;
        for (int b = b0; b < b1; b++) {
            int k = (b + half) % FFT_N; /* fftshift: -fs/2 .. +fs/2 */
            float p = g_re[k] * g_re[k] + g_im[k] * g_im[k];
            if (p > mx) mx = p;
        }
        float db = 10.0f * log10f(mx + 1.0f);
        g_col_db[c] = db;
        if (db > frame_max) {
            frame_max = db;
            peak_col = c;
        }
    }

    /* Adaptive 60 dB window: jump up instantly, drift down slowly. */
    if (!g_scale_ready) {
        g_top_db = frame_max;
        g_scale_ready = true;
    } else if (frame_max > g_top_db) {
        g_top_db = frame_max;
    } else {
        g_top_db += (frame_max - g_top_db) * 0.05f;
    }
    g_bottom_db = g_top_db - 60.0f;

    unsigned span = g_rates[g_rate_idx];
    unsigned lo = g_center_mhz > span / 2 ? g_center_mhz - span / 2 : 0;
    g_peak_mhz = lo + (unsigned)((peak_col * span) / COLS);
    g_peak_db = frame_max - g_top_db; /* <= 0, relative to the window top */
    return true;
}

/* ================================================================ */
/* rendering                                                        */
/* ================================================================ */
static void render_spectrum(void) {
    const float span = g_top_db - g_bottom_db;
    for (int x = 0; x < COLS; x++) {
        float v = (g_col_db[x] - g_bottom_db) / span;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        int h = (int)(v * (SPEC_H - 1) + 0.5f);
        int old = g_prev_h[x];
        if (old > h) {
            lcd.drawFastVLine(x, SPEC_Y + SPEC_H - old, old - h, TFT_BLACK);
        }
        if (h > 0) {
            lcd.drawFastVLine(x, SPEC_Y + SPEC_H - h, h, heat(v));
        }
        g_prev_h[x] = (uint16_t)h;
    }
}

static void render_waterfall_row(void) {
    const float span = g_top_db - g_bottom_db;
    for (int x = 0; x < COLS; x++) {
        float v = (g_col_db[x] - g_bottom_db) / span;
        g_wf_row[x] = heat(v);
    }
    memmove(g_wf, g_wf + COLS, (WF_H - 1) * COLS * sizeof(uint16_t));
    memcpy(g_wf + (WF_H - 1) * COLS, g_wf_row, COLS * sizeof(uint16_t));
    lcd.pushImage(0, WF_Y, COLS, WF_H, g_wf);
}

static void render_header(bool force) {
    static int64_t last = 0;
    static unsigned last_c = 0;
    static int last_rate = -1, last_gain = -1;
    static bool last_hold = false, last_ok = true;

    int64_t now = esp_timer_get_time();
    bool ok = g_err_count == 0;
    if (!force && now - last < 300000 && last_c == g_center_mhz && last_rate == g_rate_idx &&
        last_gain == g_gain_idx && last_hold == g_hold && last_ok == ok)
        return;
    last = now;
    last_c = g_center_mhz;
    last_rate = g_rate_idx;
    last_gain = g_gain_idx;
    last_hold = g_hold;
    last_ok = ok;

    char gain[8];
    if (g_gain_idx == 0)
        snprintf(gain, sizeof(gain), "AGC");
    else
        snprintf(gain, sizeof(gain), "G%d", g_gains[g_gain_idx]);

    char buf[64];
    if (!ok)
        snprintf(buf, sizeof(buf), "%uMHz %uM %s  CAPTURE ERR %d", g_center_mhz,
                 g_rates[g_rate_idx], gain, g_err_count);
    else if (g_hold)
        snprintf(buf, sizeof(buf), "%uMHz %uM %s  HOLD  PK%u %+.0fdB", g_center_mhz,
                 g_rates[g_rate_idx], gain, g_peak_mhz, g_peak_db);
    else
        snprintf(buf, sizeof(buf), "%uMHz %uM %s  PK%u %+.0fdB %.0ffps", g_center_mhz,
                 g_rates[g_rate_idx], gain, g_peak_mhz, g_peak_db, g_fps);

    lcd.fillRect(0, 0, SCREEN_W, HEADER_H, TFT_BLACK);
    lcd.setTextDatum(TL_DATUM);
    lcd.setTextSize(1);
    lcd.setTextColor(ok ? TFT_CYAN : TFT_RED, TFT_BLACK);
    lcd.drawString(buf, 2, 3);
}

static void draw_button(int i, const char *label, bool active) {
    int x = i * BTN_W;
    uint16_t fill = active ? TFT_MAROON : 0x0841; /* dark slate */
    lcd.fillRect(x + 1, BTN_Y + 1, BTN_W - 2, BTN_H - 2, fill);
    lcd.drawRect(x + 1, BTN_Y + 1, BTN_W - 2, BTN_H - 2, active ? TFT_RED : 0x03EF);
    lcd.setTextDatum(MC_DATUM);
    lcd.setTextSize(2);
    lcd.setTextColor(active ? TFT_WHITE : TFT_WHITE, fill);
    lcd.drawString(label, x + BTN_W / 2, BTN_Y + BTN_H / 2);
}

static void draw_buttons(void) {
    draw_button(0, g_rate_idx == 0 ? "16M" : g_rate_idx == 1 ? "40M" : "80M", false);
    draw_button(1, "-5M", false);
    draw_button(2, "+5M", false);
    draw_button(3, g_gain_idx == 0 ? "AGC" : "GAIN", g_gain_idx != 0);
    draw_button(4, "HOLD", g_hold);
    lcd.drawFastHLine(0, BTN_Y, SCREEN_W, 0x03EF);
}

static void draw_chrome(void) {
    lcd.fillScreen(TFT_BLACK);
    lcd.drawFastHLine(0, SPEC_Y + SPEC_H, SCREEN_W, 0x4208);
    for (int x = 0; x < COLS; x++) g_prev_h[x] = 0;
    memset(g_wf, 0, WF_H * COLS * sizeof(uint16_t));
    lcd.pushImage(0, WF_Y, COLS, WF_H, g_wf);
    draw_buttons();
    render_header(true);
}

/* ================================================================ */
/* touch calibration                                                */
/* ================================================================ */
static void fit_line(float *a, float *b, const uint16_t *raw, const float *scr, int n) {
    double sr = 0, ss = 0, srr = 0, srs = 0;
    for (int i = 0; i < n; i++) {
        sr += raw[i];
        ss += scr[i];
        srr += (double)raw[i] * raw[i];
        srs += (double)raw[i] * scr[i];
    }
    double den = n * srr - sr * sr;
    if (fabs(den) < 1e-9) {
        *a = 0;
        *b = (float)(ss / n);
        return;
    }
    *a = (float)((n * srs - sr * ss) / den);
    *b = (float)((ss - *a * sr) / n);
}

static float fit_sse(const uint16_t *raw, const float *scr, float a, float b, int n) {
    double s = 0;
    for (int i = 0; i < n; i++) {
        double e = a * raw[i] + b - scr[i];
        s += e * e;
    }
    return (float)s;
}

static bool calibrate_point(int px, int py, int idx, uint16_t *rx, uint16_t *ry) {
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextDatum(TC_DATUM);
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    char msg[32];
    snprintf(msg, sizeof(msg), "Tap target %d of 4", idx + 1);
    lcd.drawString(msg, SCREEN_W / 2, 8);
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.drawString("tap the crosshair - hold BOOT to skip", SCREEN_W / 2, 34);
    for (int k = 0; k < 3; k++) {
        int r = 16 - k * 5;
        lcd.drawCircle(px, py, r, TFT_RED);
        lcd.drawLine(px - r, py, px + r, py, TFT_RED);
        lcd.drawLine(px, py - r, px, py + r, TFT_RED);
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    /* Wait for a tap for as long as it takes. The live readout at the bottom
     * shows the raw conversions, so a finger that is not being detected is
     * visible immediately. */
    int64_t dbg = 0;
    for (;;) {
        xpt_raw_t r;
        bool pressed = xpt_read(&r);

        char line[80];
        snprintf(line, sizeof(line), "x=%u y=%u z1=%u z2=%u z=%d %s", (unsigned)r.x,
                 (unsigned)r.y, (unsigned)r.z1, (unsigned)r.z2, (int)r.z,
                 pressed ? "DOWN" : "up");
        lcd.setTextDatum(BL_DATUM);
        lcd.setTextSize(1);
        lcd.setTextColor(pressed ? TFT_GREEN : TFT_SILVER, TFT_BLACK);
        lcd.fillRect(0, SCREEN_H - 11, SCREEN_W, 11, TFT_BLACK);
        lcd.drawString(line, 2, SCREEN_H - 1);

        int64_t now = esp_timer_get_time();
        if (now - dbg > 1000000) {
            dbg = now;
            printf("touch raw %s\n", line);
        }

        if (pressed) {
            uint16_t x, y;
            if (xpt_touch_raw(&x, &y)) {
                *rx = x;
                *ry = y;
                printf("touch tap %d raw=%u,%u\n", idx + 1, (unsigned)x, (unsigned)y);
                /* wait for release so the next target does not inherit this tap */
                int64_t r0 = esp_timer_get_time();
                while (esp_timer_get_time() - r0 < 1500000) {
                    uint16_t a, b;
                    if (!xpt_touch_raw(&a, &b)) break;
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                return true;
            }
        }

        if (gpio_get_level(GPIO_NUM_0) == 0) {
            printf("touch cal skipped (BOOT held)\n");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

static void run_calibration(void) {
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextDatum(MC_DATUM);
    lcd.setTextSize(3);
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.drawString("Touch calibration", SCREEN_W / 2, SCREEN_H / 2 - 20);
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.drawString("4 taps", SCREEN_W / 2, SCREEN_H / 2 + 20);
    vTaskDelay(pdMS_TO_TICKS(900));

    int px[4] = {20, SCREEN_W - 20, SCREEN_W - 20, 20};
    int py[4] = {20, 20, SCREEN_H - 20, SCREEN_H - 20};
    uint16_t rx[4], ry[4];
    bool ok = true;
    for (int i = 0; i < 4; i++) {
        if (!calibrate_point(px[i], py[i], i, &rx[i], &ry[i])) {
            printf("cal: timeout at target %d\n", i);
            ok = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    if (!ok) {
        /* Never persist a calibration built from missing samples: that leaves
         * the UI alive but every tap mapped to nonsense. Fall back to a rough
         * linear map so the screen still responds, and stay uncalibrated so
         * the next boot retries properly. */
        g_cal = {};
        g_cal.sx_a = 320.0f / 3600.0f;
        g_cal.sx_b = -20.0f;
        g_cal.sy_a = 240.0f / 3600.0f;
        g_cal.sy_b = -20.0f;
        g_cal.swapped_x = false;
        g_cal.swapped_y = false;
        g_cal.valid = true;
        xpt_cal_apply(&g_cal);
        lcd.fillScreen(TFT_BLACK);
        lcd.setTextDatum(MC_DATUM);
        lcd.setTextSize(2);
        lcd.setTextColor(TFT_RED, TFT_BLACK);
        lcd.drawString("touch cal failed", SCREEN_W / 2, SCREEN_H / 2 - 16);
        lcd.setTextColor(TFT_SILVER, TFT_BLACK);
        lcd.drawString("using fallback map", SCREEN_W / 2, SCREEN_H / 2 + 12);
        vTaskDelay(pdMS_TO_TICKS(1500));
        return;
    }

    float sx[4] = {(float)px[0], (float)px[1], (float)px[2], (float)px[3]};
    float sy[4] = {(float)py[0], (float)py[1], (float)py[2], (float)py[3]};

    /* Fit each screen axis against both raw channels, then pick the pair with
     * the smaller total error. Screen X and screen Y must come from different
     * raw channels; deriving both from one channel was the old bug. */
    float ax1, bx1, ax2, bx2, ay1, by1, ay2, by2;
    fit_line(&ax1, &bx1, rx, sx, 4);
    fit_line(&ax2, &bx2, ry, sx, 4);
    fit_line(&ay1, &by1, rx, sy, 4);
    fit_line(&ay2, &by2, ry, sy, 4);
    float e_sx_rx = fit_sse(rx, sx, ax1, bx1, 4);
    float e_sx_ry = fit_sse(ry, sx, ax2, bx2, 4);
    float e_sy_rx = fit_sse(rx, sy, ay1, by1, 4);
    float e_sy_ry = fit_sse(ry, sy, ay2, by2, 4);
    float pair_a = e_sx_rx + e_sy_ry; /* x from rawX, y from rawY */
    float pair_b = e_sx_ry + e_sy_rx; /* x from rawY, y from rawX */

    xpt_cal_t cal = {};
    if (pair_b < pair_a) {
        cal.sx_a = ax2;
        cal.sx_b = bx2;
        cal.swapped_x = true;
        cal.sy_a = ay1;
        cal.sy_b = by1;
        cal.swapped_y = true;
    } else {
        cal.sx_a = ax1;
        cal.sx_b = bx1;
        cal.swapped_x = false;
        cal.sy_a = ay2;
        cal.sy_b = by2;
        cal.swapped_y = false;
    }
    cal.valid = true;
    g_cal = cal;
    xpt_cal_apply(&cal);
    xpt_cal_save(&cal);
    printf("cal: raw (%u,%u)(%u,%u)(%u,%u)(%u,%u)\n", rx[0], ry[0], rx[1], ry[1], rx[2], ry[2],
           rx[3], ry[3]);
    printf("cal: sx=%.4f*%s%+.1f sy=%.4f*%s%+.1f err A=%.0f B=%.0f\n", cal.sx_a,
           cal.swapped_x ? "ry" : "rx", cal.sx_b, cal.sy_a, cal.swapped_y ? "rx" : "ry", cal.sy_b,
           pair_a, pair_b);
}

/* ================================================================ */
/* actions                                                          */
/* ================================================================ */
static void apply_gain(void) {
    if (g_gain_idx == 0)
        sdr_gain_hardware();
    else
        sdr_gain_manual((unsigned)g_gains[g_gain_idx]);
}

static void set_center(int delta) {
    int f = (int)g_center_mhz + delta;
    if (f < 2300) f = 2300;
    if (f > 2600) f = 2600;
    if ((unsigned)f == g_center_mhz) return;
    g_center_mhz = (unsigned)f;
    g_tuned = false;
}

static void on_press(int sx, int sy) {
    if (sy >= BTN_Y) {
        int i = sx / BTN_W;
        if (i < 0) i = 0;
        if (i > 4) i = 4;
        switch (i) {
            case 0:
                g_rate_idx = (g_rate_idx + 1) % 3;
                break;
            case 1:
                set_center(-5);
                break;
            case 2:
                set_center(+5);
                break;
            case 3:
                g_gain_idx = (g_gain_idx + 1) % 6;
                apply_gain();
                break;
            case 4:
                g_hold = !g_hold;
                break;
        }
        draw_buttons();
        render_header(true);
    } else if (sy >= SPEC_Y && sy < SPEC_Y + SPEC_H) {
        /* tapping the spectrum retunes to that frequency */
        unsigned span = g_rates[g_rate_idx];
        unsigned lo = g_center_mhz > span / 2 ? g_center_mhz - span / 2 : 0;
        int mhz = (int)lo + (sx * (int)span) / SCREEN_W;
        int f = mhz;
        if (f < 2300) f = 2300;
        if (f > 2600) f = 2600;
        g_center_mhz = (unsigned)f;
        g_tuned = false;
        render_header(true);
    }
}

/* ================================================================ */
/* boot                                                             */
/* ================================================================ */
static bool alloc_buffers(void) {
    g_re = (float *)heap_caps_malloc(FFT_N * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_im = (float *)heap_caps_malloc(FFT_N * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_win = (float *)heap_caps_malloc(FFT_N * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_tw_cos =
        (float *)heap_caps_malloc(FFT_N / 2 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_tw_sin =
        (float *)heap_caps_malloc(FFT_N / 2 * sizeof(float), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_wf = (uint16_t *)heap_caps_malloc(WF_H * COLS * sizeof(uint16_t),
                                        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return g_re && g_im && g_win && g_tw_cos && g_tw_sin && g_wf;
}

extern "C" void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    if (!display_begin()) {
        printf("display init failed\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    lcd.setTextDatum(MC_DATUM);
    lcd.setTextSize(3);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.drawString("ESP-SDR", SCREEN_W / 2, 90);
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_SILVER, TFT_BLACK);
    lcd.drawString("CYD 320x240  raw I/Q spectrum", SCREEN_W / 2, 124);

    if (!alloc_buffers()) {
        lcd.setTextColor(TFT_RED, TFT_BLACK);
        lcd.drawString("out of memory", SCREEN_W / 2, 160);
        printf("out of memory\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    fft_tables();

    xpt_init();
    if (!xpt_cal_load(&g_cal)) {
        run_calibration();
    } else {
        xpt_cal_apply(&g_cal);
        printf("cal loaded sx=%.4f*%s%+.1f sy=%.4f*%s%+.1f\n", g_cal.sx_a,
               g_cal.swapped_x ? "ry" : "rx", g_cal.sx_b, g_cal.sy_a,
               g_cal.swapped_y ? "rx" : "ry", g_cal.sy_b);
    }

    printf("ESP-SDR CYD: bringing up radio\n");
    lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
    lcd.drawString("radio init...", SCREEN_W / 2, 160);
    sdr_init();
    sdr_tune_mhz(g_center_mhz);
    g_tuned = true;
    apply_gain();
    printf("radio ready at %u MHz, max gain %u\n", sdr_frequency_mhz(), sdr_gain_max());

    draw_chrome();

    int64_t prev_frame = 0;
    bool touch_down = false;
    int64_t header_press = 0;

    for (;;) {
        int64_t now = esp_timer_get_time();

        /* ---- touch ---- */
        uint16_t rx, ry;
        bool down = xpt_touch_raw(&rx, &ry);
        int sx = 0, sy = 0;
        if (down && g_cal.valid) {
            xpt_map(rx, ry, &sx, &sy);
        } else {
            down = false;
        }

        if (down && !touch_down && g_cal.valid) {
            on_press(sx, sy);
            if (sy <= HEADER_H) header_press = now;
        }
        if (!down) header_press = 0;
        if (down && header_press && now - header_press > 1500000) {
            /* long press on the header re-runs calibration */
            header_press = 0;
            run_calibration();
            draw_chrome();
            g_tuned = false;
        }
        touch_down = down;

        /* ---- radio ---- */
        if (!g_hold) {
            if (!g_tuned) {
                if (sdr_tune_mhz(g_center_mhz)) {
                    g_tuned = true;
                    printf("tuned %u MHz\n", g_center_mhz);
                } else {
                    printf("tune failed %u MHz\n", g_center_mhz);
                }
            }
            bool ok = process_frame();
            if (ok) {
                g_frames++;
                g_err_count = 0;
                snprintf(g_status, sizeof(g_status), "RUN");
                render_spectrum();
                render_waterfall_row();
            } else {
                g_fail_total++;
                if (g_err_count < 999) g_err_count++;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        int64_t dt = now - prev_frame;
        if (dt > 0 && !g_hold) {
            float fps = 1000000.0f / (float)dt;
            g_fps = g_fps == 0 ? fps : g_fps * 0.9f + fps * 0.1f;
        }
        prev_frame = now;

        render_header(false);

        /* Slow status line; only useful while bringing the radio up. */
        static int64_t dbg_last = 0;
        if (now - dbg_last > 5000000) {
            dbg_last = now;
            printf("status: %lu fps, %lu failed captures, %u free bytes\n",
                   (unsigned long)(g_fps + 0.5f), (unsigned long)g_fail_total,
                   (unsigned)esp_get_free_heap_size());
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
