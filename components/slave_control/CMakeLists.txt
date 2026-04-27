idf_component_register(SRCS "slave_uart.c"
                       INCLUDE_DIRS "." 
                       REQUIRES esp_driver_gpio esp_driver_uart)
