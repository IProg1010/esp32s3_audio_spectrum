idf_component_register(SRCS "slave_uart.c"
                       INCLUDE_DIRS "." 
                       REQUIRES driver esp_driver_uart)
