#include "display.hpp"

#include <cstdio>

CydDisplay::CydDisplay() {
    {
        auto cfg = _bus.config();
        cfg.spi_host = HSPI_HOST;
        cfg.spi_mode = 0;
        cfg.freq_write = 80000000;
        cfg.freq_read = 16000000;
        cfg.spi_3wire = false; /* MISO (GPIO12) is wired on this board. */
        cfg.use_lock = true;
        cfg.dma_channel = SPI_DMA_CH_AUTO;
        cfg.pin_sclk = 14;
        cfg.pin_mosi = 13;
        cfg.pin_miso = 12;
        cfg.pin_dc = 2;
        _bus.config(cfg);
        _panel.setBus(&_bus);
    }
    {
        auto cfg = _panel.config();
        cfg.pin_cs = 15;
        cfg.pin_rst = -1;
        cfg.pin_busy = -1;
        cfg.panel_width = 240;
        cfg.panel_height = 320;
        cfg.memory_width = 240;
        cfg.memory_height = 320;
        cfg.offset_x = 0;
        cfg.offset_y = 0;
        cfg.offset_rotation = 0;
        cfg.readable = true;
        cfg.invert = false;   /* scanner runs invertDisplay(false) */
        cfg.rgb_order = false;
        cfg.bus_shared = false;
        _panel.config(cfg);
    }
    {
        auto cfg = _light.config();
        cfg.pin_bl = 21;      /* CYD backlight */
        cfg.freq = 12000;
        cfg.pwm_channel = 7;
        cfg.invert = false;
        _light.config(cfg);
        _panel.setLight(&_light);
    }
    setPanel(&_panel);
}

CydDisplay lcd;

bool display_begin(void) {
    if (!lcd.init()) return false;
    /* Panel is 240x320 native. Rotations 1 and 3 are the landscape modes
     * (320x240); rotation 6 is portrait and was cropping the UI. */
    lcd.setRotation(1);
    lcd.setBrightness(200);
    lcd.fillScreen(TFT_BLACK);
    printf("display: %dx%d rot=%d\n", (int)lcd.width(), (int)lcd.height(), (int)lcd.getRotation());
    return true;
}
