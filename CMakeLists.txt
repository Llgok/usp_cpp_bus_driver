set(semtech_source_root "${CMAKE_CURRENT_LIST_DIR}/third_party/usp")
set(swl2001_source_root "${CMAKE_CURRENT_LIST_DIR}/third_party/SWL2001")
set(
        radio_driver_root
        "${semtech_source_root}/smtc_rac_lib/radio_drivers"
)
set(
        swl2001_radio_driver_root
        "${swl2001_source_root}/lbm_lib/smtc_modem_core/radio_drivers"
)
set(sx126x_driver_dir "${radio_driver_root}/sx126x_driver/src")
set(sx127x_driver_dir "${swl2001_radio_driver_root}/sx127x_driver/src")
set(lr11xx_driver_dir "${radio_driver_root}/lr11xx_driver/src")
set(lr20xx_driver_inc_dir "${radio_driver_root}/lr20xx_driver/inc")
set(lr20xx_driver_src_dir "${radio_driver_root}/lr20xx_driver/src")

set(
        sx126x_driver_sources
        "${sx126x_driver_dir}/lr_fhss_mac.c"
        "${sx126x_driver_dir}/sx126x.c"
        "${sx126x_driver_dir}/sx126x_bpsk.c"
        "${sx126x_driver_dir}/sx126x_driver_version.c"
        "${sx126x_driver_dir}/sx126x_lr_fhss.c"
)

set(
        sx127x_driver_sources
        "${sx127x_driver_dir}/sx127x.c"
)

set(
        lr11xx_driver_sources
        "${lr11xx_driver_dir}/lr11xx_bootloader.c"
        "${lr11xx_driver_dir}/lr11xx_crypto_engine.c"
        "${lr11xx_driver_dir}/lr11xx_driver_version.c"
        "${lr11xx_driver_dir}/lr11xx_gnss.c"
        "${lr11xx_driver_dir}/lr11xx_lr_fhss.c"
        "${lr11xx_driver_dir}/lr11xx_radio.c"
        "${lr11xx_driver_dir}/lr11xx_radio_timings.c"
        "${lr11xx_driver_dir}/lr11xx_regmem.c"
        "${lr11xx_driver_dir}/lr11xx_rttof.c"
        "${lr11xx_driver_dir}/lr11xx_system.c"
        "${lr11xx_driver_dir}/lr11xx_wifi.c"
)

set(
        lr20xx_driver_sources
        "${lr20xx_driver_src_dir}/lr20xx_driver_version.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_bluetooth_le.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_bpsk.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_common.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_fifo.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_flrc.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_fsk.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_lora.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_lr_fhss.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_ook.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_oqpsk_15_4.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_wi_sun.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_wm_bus.c"
        "${lr20xx_driver_src_dir}/lr20xx_radio_z_wave.c"
        "${lr20xx_driver_src_dir}/lr20xx_regmem.c"
        "${lr20xx_driver_src_dir}/lr20xx_rttof.c"
        "${lr20xx_driver_src_dir}/lr20xx_system.c"
        "${lr20xx_driver_src_dir}/lr20xx_workarounds.c"
)

idf_component_register(
        SRCS
        "src/lr11xx/lr11xx.cpp"
        "src/lr11xx/lr11xx_hal.cpp"
        "src/lr20xx/lr20xx.cpp"
        "src/lr20xx/lr20xx_hal.cpp"
        "src/common/radio_context.cpp"
        "src/sx126x/sx126x.cpp"
        "src/sx126x/sx126x_hal.cpp"
        "src/sx127x/sx127x.cpp"
        "src/sx127x/sx127x_context.cpp"
        "src/sx127x/sx127x_hal.cpp"
        ${sx126x_driver_sources}
        ${sx127x_driver_sources}
        ${lr11xx_driver_sources}
        ${lr20xx_driver_sources}
        INCLUDE_DIRS
        "src"
        "${semtech_source_root}"
        "${sx126x_driver_dir}"
        "${sx127x_driver_dir}"
        "${lr11xx_driver_dir}"
        "${lr20xx_driver_inc_dir}"
        REQUIRES
        "cpp_bus_driver"
        PRIV_REQUIRES
        "esp_driver_gpio"
        "esp_system"
        "esp_timer"
        "freertos"
)

target_compile_definitions(
        ${COMPONENT_LIB}
        PRIVATE
        LR11XX_DISABLE_WARNINGS
        SX1272
        SX1276
)
