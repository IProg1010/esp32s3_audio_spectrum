set(screen_files "main.c" "welcome.c")

idf_component_register(SRCS "ili9486.c" "./screens/main.c" "./screens/welcome.c" "./input/input_manager.c"
                       INCLUDE_DIRS "." "./screens" "./input"
                       REQUIRES driver esp_lcd esp_lvgl_port)
