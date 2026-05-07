idf_component_register(SRCS "slave_uart.c" "./wch-isp-v0.5.0/wch-isp.c" "wchisp.c"
                       INCLUDE_DIRS "." "./wch-isp-v0.5.0"
                       REQUIRES esp_driver_gpio esp_driver_uart)

target_add_binary_data(${COMPONENT_LIB} "../../slave_controller/output_files/flash_slave.bin" BINARY)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers -Wno-shift-count-overflow -Wno-format -Wno-implicit-fallthrough)