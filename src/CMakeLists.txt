idf_component_register(
    SRCS "main.cpp" "display.cpp" "sdr_rx.c" "rx_recalibration.c"
         "rx_calibration_state.c" "xpt2046.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_gpio esp_driver_spi esp_phy esp_wifi esp_event esp_timer nvs_flash heap)

# esp-sdr's receive-only retune path replaces the PHY's channel setup so a
# retune never runs the transmitting loopback IQ calibration.
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=chip_v7_set_chan_ana")

# Fail the link if static data would land inside the 64 KiB capture aperture.
target_link_options(${COMPONENT_LIB} INTERFACE
    "-T${CMAKE_CURRENT_LIST_DIR}/sram_guard.ld")
