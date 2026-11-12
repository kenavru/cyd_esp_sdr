/* Original ESP32 RX-only backend.
 * Register sequence derived from ESP-IDF's ESP32 librftest.a: adctrig/mac_init.
 * Trimmed from ESPARGOS/esp-sdr main/targets/esp32/receiver.c (GPL-3.0):
 * the serial burst protocol, spectrum packetizer and command shell are gone;
 * sdr_rx.h exposes the same tune/capture/gain/filter operations directly.
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdr_rx.h"
#include "rx_recalibration.h"
#include "rx_tuning.h"
#include "rx_lo.h"
#include "rx_bandwidth.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "soc/soc.h"
#include "soc/rtc.h"
#include "soc/dport_reg.h"
#include "soc/dport_access.h"
#include "heap_memory_layout.h"

/* Reserve both SRAM banks, including their instruction aliases. Mode 2 only
 * exposes 32 KiB to MAC; mode 3 exposes the entire 64 KiB capture aperture. */
SOC_RESERVE_MEMORY_REGION(0x3ffe8000, 0x3fff8000, esp32_rf_dump);
#define DUMP_CTRL   0x60033d90u
#define DUMP_STATUS 0x60033d94u
#define DUMP_BYTES  0x60033dc4u
#define RX_GAIN     0x3ff5c02cu
#define ADC_CTRL    0x3ff460a0u
#define ADC_ON_FORCE (BIT(9) | BIT(8)) /* Force-enable and ADC-on value. */
#define CAPACITY    16384u
#define MAX_SAMPLES (CAPACITY - 4)
#define SENTINEL    0xa55aa55au
static uint32_t *const samples = (void *)0x3ffe8000;
static unsigned gain_max, gain_code = 40;
static bool hardware_agc = true;
static unsigned frequency_mhz = 2412;
/* Original ESP32 BBTOP RX filter: registers 1/2, seven-bit capacitor DAC.
 * Preserve bit 7 and restore PHY calibration before any retune or transfer. */
static int rx_filter = -1;
extern unsigned rom_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void rom_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
static unsigned filter_saved[2];
static void filter_apply(void) {
    for (unsigned j = 0; j < 2; j++) {
        filter_saved[j] = rom_chip_i2c_readReg(0x67, 1, 1 + j);
        if (rx_filter >= 0)
            rom_chip_i2c_writeReg(0x67, 1, 1 + j, (filter_saved[j] & ~127u) | (unsigned)rx_filter);
    }
    esp_rom_delay_us(10);
}
static void filter_restore(void) {
    if (rx_filter >= 0)
        for (unsigned j = 0; j < 2; j++) rom_chip_i2c_writeReg(0x67, 1, 1 + j, filter_saved[j]);
}
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(int);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(int);

static void tune_rx(unsigned mhz);
static void apply_gain(void);
static bool rx_ready;
static void prepare_rx(void) {
    if (!rx_ready || rx_recalibration_stale()) {
        tune_rx(frequency_mhz);
        rx_ready = true;
    }
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    /* PBUS work-mode setup releases a forced RX gain; apply it afterwards. */
    apply_gain();
    if (rx_lo_select(rx_lo_plan(frequency_mhz).alternate)) esp_rom_delay_us(3000);
}

/* Clock selectors: 0=16 MS/s, 1=80 MS/s, 2=40 MS/s. Hardware sampling,
 * not software decimation. Rates inferred from capture-duration slopes. */
static bool acquire_iq(unsigned n, unsigned source, unsigned clock, unsigned *capture_us) {
    if (n < 256 || n > MAX_SAMPLES || source > 3 || clock > 2) return false;
    if (rx_recalibration_stale()) prepare_rx();
    REG_WRITE(DUMP_CTRL, 0);
    for (unsigned j = 0; j < CAPACITY; j++) samples[j] = SENTINEL;
    filter_apply();
    /* RX clock forcing alone does not keep the ADC on. PHY gating otherwise
     * inserts short runs of the inactive converter's fixed I/Q value. */
    uint32_t adc_on = DPORT_REG_READ(ADC_CTRL) & ADC_ON_FORCE;
    DPORT_REG_WRITE(ADC_CTRL, DPORT_REG_READ(ADC_CTRL) | ADC_ON_FORCE);
    esp_rom_delay_us(2);
    uint32_t owner = DPORT_REG_READ(DPORT_IRAM_DRAM_AHB_SEL_REG);
    uint32_t byte_select = REG_READ(DUMP_BYTES);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,
                    (owner & ~DPORT_MAC_DUMP_MODE_M) | (3 << DPORT_MAC_DUMP_MODE_S));
    REG_WRITE(DUMP_BYTES, 0x03020100); /* Four distinct bytes, as in mac_init. */
    uint32_t ctrl = n | (source << 20) | ((clock == 0) << 16) | ((clock == 2) << 15);
    REG_WRITE(DUMP_CTRL, ctrl | BIT(31));
    int64_t start = esp_timer_get_time();
    REG_SET_BIT(DUMP_CTRL, BIT(19));
    REG_CLR_BIT(DUMP_CTRL, BIT(19));
    while (!(REG_READ(DUMP_CTRL) & BIT(18)) && esp_timer_get_time() - start < 20000) {}
    uint32_t result = REG_READ(DUMP_CTRL), status = REG_READ(DUMP_STATUS);
    unsigned elapsed = esp_timer_get_time() - start;
    REG_WRITE(DUMP_CTRL, 0);
    esp_rom_delay_us(10);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG, owner);
    REG_WRITE(DUMP_BYTES, byte_select);
    DPORT_REG_WRITE(ADC_CTRL, (DPORT_REG_READ(ADC_CTRL) & ~ADC_ON_FORCE) | adc_on);
    filter_restore();
    if (!(result & BIT(18)) || (status & 0x7fff) != n) {
        ESP_LOGE("sdr", "capture timeout (result %08" PRIx32 ", status %08" PRIx32 ")",
                 result, status);
        return false;
    }
    for (unsigned j = 0; j < CAPACITY; j++) {
        if ((j < n && samples[j] == SENTINEL) || (j >= n && samples[j] != SENTINEL)) {
            ESP_LOGE("sdr", "capture memory mismatch at %u", j);
            return false;
        }
    }
    *capture_us = elapsed;
    return true;
}

static void apply_gain(void) {
    uint32_t gain = REG_READ(RX_GAIN);
    if (hardware_agc)
        REG_CLR_BIT(RX_GAIN, BIT(23));
    else
        REG_WRITE(RX_GAIN, (gain & 0x7fffff) | (gain_code << 24) | BIT(23));
}

extern void set_chanfreq(unsigned mhz, unsigned mode);
extern void rom_set_rf_freq_offset(unsigned crystal, unsigned mhz, int offset);

static void tune_rx(unsigned mhz) {
    static unsigned calibrated_mhz;
    if (calibrated_mhz != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    rx_lo_plan_t plan = rx_lo_plan(mhz);
    rx_lo_select(false);
    bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484;
    /* Calibrate on a real channel before bypassing the channel-number mapping.
     * Use the PHY entry point even for repeated requests: the Wi-Fi API can
     * skip a channel it thinks is already selected after a direct PLL retune. */
    set_chanfreq(channel ? mhz : 2412, 0);
    if (!channel) {
        unsigned xtal = rtc_clk_xtal_freq_get();
        /* Channel setup pins its capacitor code. Release it before the ROM
         * retune, otherwise the SDM changes but the PLL can remain unlocked. */
        unsigned cap = ram_chip_i2c_readReg(0x62, 1, 0);
        ram_chip_i2c_writeReg(0x62, 1, 0, cap & ~0x80u);
        /* This ROM uses 1/1024 MHz offsets, unlike the newer kHz API. */
        int offset = (plan.offset_khz * 1024 + 500) / 1000;
        rom_set_rf_freq_offset(xtal == 26 ? 1 : xtal == 24 ? 2 : 0, plan.mhz, offset);
    }
}

/* ------------------------------------------------------------------ */

void sdr_init(void) {
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    prepare_rx();
    gain_max = (REG_READ(RX_GAIN) >> 8) & 127;
    REG_CLR_BIT(RX_GAIN, BIT(23)); /* Hardware AGC is the default. */
    esp_log_level_set("sdr", ESP_LOG_ERROR);
}

bool sdr_tune_mhz(unsigned mhz) {
    if (!rx_frequency_valid(mhz)) return false;
    frequency_mhz = mhz;
    /* TUNE in the original protocol invalidates rx_ready, then re-prepares. */
    rx_ready = false;
    prepare_rx();
    return true;
}

unsigned sdr_frequency_mhz(void) { return frequency_mhz; }

bool sdr_acquire(unsigned n, sdr_clock_t clock, unsigned *elapsed_us) {
    return acquire_iq(n, 0, (unsigned)clock, elapsed_us);
}

const uint32_t *sdr_iq(void) { return samples; }

unsigned sdr_gain_max(void) { return gain_max; }

void sdr_gain_hardware(void) {
    hardware_agc = true;
    apply_gain();
}

void sdr_gain_manual(unsigned code) {
    if (code > gain_max) code = gain_max;
    hardware_agc = false;
    gain_code = code;
    apply_gain();
}

bool sdr_gain_is_hardware(void) { return hardware_agc; }

void sdr_set_filter(int dcap) {
    if (dcap >= 0 && dcap > 127) return;
    rx_filter = dcap;
}

int sdr_get_filter(void) { return rx_filter; }
