/* Bit-bang XPT2046 driver for the ESP32-2432S028.
 *
 * The touch controller sits on its own pins (SCLK 25 / MOSI 32 / MISO 39 /
 * CS 33) and is not wired to the display's HSPI bus. Direct bit-bang reads
 * are the configuration that was verified stable on this board.
 */
#include "xpt2046.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "nvs.h"
#include "nvs_flash.h"

#define T_SCLK 25
#define T_MOSI 32
#define T_MISO 39
#define T_CS   33

#define Z_TOUCH   250
#define CAL_MAGIC 0x43594433u /* "CYD3" - bumped to invalidate a bad blob */

typedef struct {
    uint32_t magic;
    xpt_cal_t cal;
} cal_blob_t;

static xpt_cal_t s_cal;

/* The XPT2046 is clocked at ~125 kHz (4 us per half period), matching the
 * timing that was verified on this board. The differential Z conversion
 * (PD=00 in the command byte) is sensitive to a fast clock. */
static inline void dly(void) { esp_rom_delay_us(4); }

static uint16_t xpt_read_cmd(uint8_t cmd) {
    uint16_t v = 0;
    gpio_set_level(T_CS, 0);
    esp_rom_delay_us(10);
    for (int b = 7; b >= 0; b--) {
        gpio_set_level(T_SCLK, 0);
        gpio_set_level(T_MOSI, (cmd >> b) & 1);
        dly();
        gpio_set_level(T_SCLK, 1);
        dly();
    }
    for (int b = 0; b < 16; b++) {
        gpio_set_level(T_SCLK, 0);
        dly();
        v = (uint16_t)((v << 1) | (gpio_get_level(T_MISO) & 1));
        gpio_set_level(T_SCLK, 1);
        dly();
    }
    gpio_set_level(T_CS, 1);
    return (v >> 3) & 0xFFF;
}

bool xpt_read(xpt_raw_t *out) {
    /* Pressure is measured differentially (SER/DFR = 1, PD = 01): with SER
     * mode the Z nodes read 0/4095 and pressure is never detected. Command
     * bytes match the upstream XPT2046 driver (0xB1 / 0xC1 / 0xD0 / 0x90). */
    uint16_t z1 = xpt_read_cmd(0xB1);
    uint16_t z2 = xpt_read_cmd(0xC1);
    out->z1 = z1;
    out->z2 = z2;
    int32_t z = (int32_t)z1 + 4095 - (int32_t)z2;
    out->z = (z > 0 && z < 4000) ? z : 0;
    out->x = xpt_read_cmd(0xD0);
    out->y = xpt_read_cmd(0x90);
    return out->z > Z_TOUCH;
}

void xpt_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ull << T_SCLK) | (1ull << T_MOSI) | (1ull << T_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    io.pin_bit_mask = 1ull << T_MISO;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);

    gpio_set_level(T_SCLK, 0);
    gpio_set_level(T_MOSI, 0);
    gpio_set_level(T_CS, 1);

    /* BOOT button (GPIO0) is the escape hatch from the calibration wizard. */
    gpio_config_t boot_io = {
        .pin_bit_mask = 1ull << 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&boot_io);

    s_cal.valid = false;
    xpt_cal_load(&s_cal);
    xpt_cal_apply(&s_cal);
}

bool xpt_touch_raw(uint16_t *rx, uint16_t *ry) {
    uint32_t sx = 0, sy = 0;
    int n = 0;
    for (int i = 0; i < 5; i++) {
        xpt_raw_t r;
        if (xpt_read(&r) && i >= 1) {
            sx += r.x;
            sy += r.y;
            n++;
        }
    }
    if (n < 3) return false;
    *rx = (uint16_t)(sx / n);
    *ry = (uint16_t)(sy / n);
    return true;
}

bool xpt_cal_load(xpt_cal_t *cal) {
    nvs_handle_t h;
    if (nvs_open("sdrtouch", NVS_READONLY, &h) != ESP_OK) return false;
    cal_blob_t blob;
    size_t len = sizeof(blob);
    esp_err_t e = nvs_get_blob(h, "cal", &blob, &len);
    nvs_close(h);
    if (e != ESP_OK || len != sizeof(blob) || blob.magic != CAL_MAGIC) return false;
    *cal = blob.cal;
    cal->valid = true;
    return true;
}

void xpt_cal_save(const xpt_cal_t *cal) {
    nvs_handle_t h;
    if (nvs_open("sdrtouch", NVS_READWRITE, &h) != ESP_OK) return;
    cal_blob_t blob = {.magic = CAL_MAGIC, .cal = *cal};
    blob.cal.valid = true;
    if (nvs_set_blob(h, "cal", &blob, sizeof(blob)) == ESP_OK) nvs_commit(h);
    nvs_close(h);
}

void xpt_cal_apply(const xpt_cal_t *cal) { s_cal = *cal; }

void xpt_map(uint16_t rx, uint16_t ry, int *sx, int *sy) {
    uint16_t cx = s_cal.swapped_x ? ry : rx;
    uint16_t cy = s_cal.swapped_y ? rx : ry;
    *sx = (int)(s_cal.sx_a * (float)cx + s_cal.sx_b + 0.5f);
    *sy = (int)(s_cal.sy_a * (float)cy + s_cal.sy_b + 0.5f);
}
