/* Receive-only retune calibration, ESP32 branch of ESPARGOS/esp-sdr
 * main/common/rx_recalibration.c (GPL-3.0).
 *
 * These private ABIs and offsets belong to the PHY archive. Calibration
 * normally tunes fixed reference channels internally; the frequency override
 * is restricted to the synchronous measurement interval so boot and ordinary
 * channel setup keep using the original functions. */
#include "sdkconfig.h"
#include "rx_recalibration.h"
#include <stdint.h>
#include "soc/soc.h"
#include "soc/rtc.h"
#include "rx_lo.h"

static unsigned measurement_mhz;

static void release_manual_gain(void) {
    /* Previous manual gain must not override the measurement's gain sweep. */
    REG_CLR_BIT(0x3ff5c02cu, 1u << 23);
}

extern unsigned char phy_param[];
extern void __real_chip_v7_set_chan_ana(unsigned);
extern void rom_set_rf_freq_offset(unsigned, unsigned, int);
extern void set_rx_gain_table(unsigned, unsigned);
extern uint32_t chip7_sleep_params[];

static void measurement_tune(void) {
    rx_lo_plan_t plan = rx_lo_plan(measurement_mhz);
    unsigned crystal = rtc_clk_xtal_freq_get();
    unsigned cap = ram_chip_i2c_readReg(0x62, 1, 0);
    ram_chip_i2c_writeReg(0x62, 1, 0, cap & ~0x80u);
    rom_set_rf_freq_offset(crystal == 26 ? 1 : crystal == 24 ? 2 : 0, plan.mhz,
                           (plan.offset_khz * 1024 + 500) / 1000);
    rx_lo_select(plan.alternate);
}

void __wrap_chip_v7_set_chan_ana(unsigned channel) {
    __real_chip_v7_set_chan_ana(channel);
    if (measurement_mhz) measurement_tune();
}

void rx_recalibrate_locked(unsigned mhz) {
    measurement_mhz = mhz;
    release_manual_gain();
    measurement_tune();
    /* Rebuild BT/Wi-Fi DC and gain memory, retaining IQ-valid bit 0x400. */
    chip7_sleep_params[0] &= ~0x20320u;
    set_rx_gain_table(rx_lo_plan(mhz).mhz, 0);
    measurement_mhz = 0;
}
