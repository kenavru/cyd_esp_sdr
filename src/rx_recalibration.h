#pragma once

/* Receiver and capture must be stopped. Measure fresh RX DC at mhz and rebuild
 * the gain tables while retaining the IQ correction established at startup.
 * Retuning must not start a transmitting loopback IQ calibration. Caller restores
 * channel, filter, RX power and gain settings afterwards. Stock startup and
 * background PHY calibration remain outside this receive-only retune path. */
void rx_recalibrate(unsigned mhz);

#include <stdbool.h>
/* True if the requested-LO DC calibration is missing or has been replaced.
 * Check at capture preparation, including when the requested LO is unchanged.
 * Recovery is deferred until capture has stopped; this is not a capture lock. */
bool rx_recalibration_stale(void);
