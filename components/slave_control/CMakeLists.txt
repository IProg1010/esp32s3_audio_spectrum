idf_component_register(SRCS "slave_uart.c" "wchisp.c"
                       INCLUDE_DIRS "." 
                       REQUIRES esp_driver_gpio esp_driver_uart)

target_add_binary_data(${COMPONENT_LIB} "../../slave_controller/output_files/flash_slave.bin" BINARY)