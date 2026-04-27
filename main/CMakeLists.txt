idf_component_register(SRCS "audio_spectrum.c"
                       INCLUDE_DIRS "." 
                       REQUIRES driver display adc_input dac_output slave_control esp_adc esp_driver_i2s)

target_add_binary_data(${COMPONENT_LIB} "../slave_controller/output_files/flash_slave.bin" BINARY)
