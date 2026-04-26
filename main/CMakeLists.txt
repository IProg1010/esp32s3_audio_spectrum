idf_component_register(SRCS "audio_spectrum.c"
                       INCLUDE_DIRS "." 
                       REQUIRES driver display adc_input dac_output slave_control esp_adc esp_driver_i2s)
