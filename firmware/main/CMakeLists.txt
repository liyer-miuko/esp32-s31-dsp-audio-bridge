idf_component_register(SRCS "uac_bridge_main.c" "uac_usb_intr.c" "uac2_audio.c" "i2s_out_v2.c" "i2s1_out.c" "pcm_dma_stage.c" "a2dp_audio_int.c" "dsp_audio.c"
                            "dsp_config.c" "dsp_filter.c" "dsp_engine.c" "dsp_biquad_s31.S" "serial_proto.c" "hw_mute.c"
                            "ble_tuning.c" "bt_codec_guard.c" "audio_input_budget.c" "amp_profile.c" "amp_control.c"
                       INCLUDE_DIRS "."
                       REQUIRES amp_json esp_driver_i2s esp_driver_i2c esp_driver_uart esp_driver_usb_serial_jtag esp_driver_gpio esp_hw_support nvs_flash bt esp_timer esp_partition spi_flash heap)

# S31 hardware-loop biquads: IRAM leaf functions, no runtime probe.
# The DSP chain runs per sample on core1; build it optimised even though the
# project default is -Og (Debug). Keep the rest of the project unchanged.
set_source_files_properties("dsp_engine.c" "dsp_filter.c" "dsp_audio.c"
    PROPERTIES COMPILE_OPTIONS "-O2")

# 0.99: LDAC decode saturated core1 in steady stereo playback with -Og:
# codec queue drops and I2S starvation, despite a bypassed DSP. Optimise
# only the existing fork's decoder translation unit; no vendor override,
# DSP/compiler-global changes, extra buffers or fast-math assumptions.
idf_component_get_property(ldac_bt_dir bt COMPONENT_DIR)
idf_component_get_property(ldac_bt_lib bt COMPONENT_LIB)
set_source_files_properties(
    "${ldac_bt_dir}/host/bluedroid/external/libldac-dec/src/ldaclib.c"
    "${ldac_bt_dir}/host/bluedroid/external/libldac-dec/src/ldacBT.c"
    "${ldac_bt_dir}/host/bluedroid/stack/a2dp/a2dp_vendor_ldacbt_decoder.c"
    "${ldac_bt_dir}/host/bluedroid/external/libfreeaptx/main/freeaptx.c"
    "${ldac_bt_dir}/host/bluedroid/stack/a2dp/a2dp_vendor_aptx_decoder.c"
    TARGET_DIRECTORY ${ldac_bt_lib}
    PROPERTIES COMPILE_OPTIONS "-O2")

# 0.141: all five codec units above run on core0. Do not change the frozen
# media worker, its affinity/queues, global optimisation or floating-point rules.

# 0.85 read-only DMA-tail probe, scoped to this source; no driver override.
idf_component_get_property(i2s_component_dir esp_driver_i2s COMPONENT_DIR)
set_property(SOURCE "i2s_out_v2.c" APPEND PROPERTY INCLUDE_DIRECTORIES "${i2s_component_dir}")

# Provide the project tusb_config.h to the espressif/tinyusb component.
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PRIVATE "${CMAKE_CURRENT_LIST_DIR}")

# 0.91: select level 2 only for the HS USB source, through an app-owned wrapper.
# The pinned IDF and managed TinyUSB sources remain byte-for-byte unchanged.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_intr_alloc")
