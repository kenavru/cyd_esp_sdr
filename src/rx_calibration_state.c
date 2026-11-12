/* DC-generation tracking, ESP32 branch of ESPARGOS/esp-sdr
 * main/common/rx_calibration_state.c (GPL-3.0).
 *
 * Count actual DC measurements, not a temperature stamp: the PHY can replace
 * a DC table without changing that stamp. These Xtensa archives resolve their
 * internal DC calls locally, so --wrap cannot intercept them; compare the
 * exported DC tables instead. Sizes are from the PHY archive, not offsets. */
#include "sdkconfig.h"
#include "rx_recalibration.h"
#include "esp_private/phy.h"
#include <sys/lock.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define RXBB_DC_BYTES 16
#define RXRF_DC_BYTES 136
extern unsigned char phy_rxbb_dc[RXBB_DC_BYTES];
extern unsigned char phy_rxrf_dc[RXRF_DC_BYTES];
static unsigned char calibrated_rxbb_dc[RXBB_DC_BYTES];
static unsigned char calibrated_rxrf_dc[RXRF_DC_BYTES];
static bool calibrated;
extern void rx_recalibrate_locked(unsigned mhz);

bool rx_recalibration_stale(void) {
    _lock_t lock = phy_get_lock();
    _lock_acquire(&lock);
    bool stale = !calibrated || memcmp(phy_rxbb_dc, calibrated_rxbb_dc, RXBB_DC_BYTES) ||
                 memcmp(phy_rxrf_dc, calibrated_rxrf_dc, RXRF_DC_BYTES);
    _lock_release(&lock);
    return stale;
}

void rx_recalibrate(unsigned mhz) {
    _lock_t lock = phy_get_lock();
    _lock_acquire(&lock);
    rx_recalibrate_locked(mhz);
    memcpy(calibrated_rxbb_dc, phy_rxbb_dc, RXBB_DC_BYTES);
    memcpy(calibrated_rxrf_dc, phy_rxrf_dc, RXRF_DC_BYTES);
    calibrated = true;
    _lock_release(&lock);
}
