#pragma once

#include <LovyanGFX.hpp>

/* Explicit config for the ESP32-2432S028 (single-USB CYD) as used by the
 * working Wi-Fi/BLE scanner project:
 *   ST7789 240x320 on HSPI: MOSI 13, MISO 12, SCLK 14, CS 15, DC 2, RST -1
 *   backlight on GPIO 21 (PWM channel 7)
 * The touch controller (XPT2046) lives on its own pins and is driven by
 * xpt2046.c, so no LovyanGFX touch instance is attached here. */
class CydDisplay : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789 _panel;
    lgfx::Bus_SPI _bus;
    lgfx::Light_PWM _light;

  public:
    CydDisplay();
};

extern CydDisplay lcd;

bool display_begin(void);
