
#include "ili9486.h"
#include "display.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "esp_lvgl_port.h"

#define LCD_H_RES 240
#define LCD_V_RES 320

// display pin according to the diagram 
#define LCD_D0 39
#define LCD_D1 40
#define LCD_D2 41
#define LCD_D3 42
#define LCD_D4 45
#define LCD_D5 46
#define LCD_D6 47
#define LCD_D7 48

#define LCD_WR 4
#define LCD_DC 6
#define LCD_CS 7
#define LCD_RST 15


void init_display()
{
    init_ili9486();
}

void update_display()
{

}

void reset_display()
{
    
}

void create_button()
{

}

void click_button(int n)
{

}

void chage_focus(int n)
{

}

int16_t get_encoder_diff()
{
    return 0;
}

bool get_button_state()
{
    return false;
}

static void encoder_read_cb(lv_indev_t * indev, lv_indev_data_t * data) {
    // Получите данные от вашего контроллера здесь
    int16_t enc_diff = get_encoder_diff(); // Сколько щелчков сделал (+1, -1, 0)
    bool is_pressed = get_button_state();  // Нажата ли кнопка энкодера

    data->enc_diff = enc_diff;
    data->state = is_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}


// Глобальные хендлы (чтобы были доступны везде)
esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_io_handle_t io_h = NULL; 

void init_ili9486()
{
    // 1. Setting bus I80
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = LCD_DC,
        .wr_gpio_num = LCD_WR,
        .data_gpio_nums = { LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7 },
        .bus_width = 8,
        .max_transfer_bytes = 320 * 40 * 2, // max buffer for write to display
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    // 2. Setting IO
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = LCD_CS,
        .pclk_hz = 2 * 10000 * 1000, 
        .trans_queue_depth = 10,
        .dc_levels = { .dc_idle_level = 0, .dc_cmd_level = 0, .dc_dummy_level = 0, .dc_data_level = 1 },
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = { .pclk_idle_low = 1 }, // Вместо pclk_active_neg
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_h));

    // 3. Create object for lcd display
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    // Use ST7789 driver for ILI9486
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_h, &panel_config, &panel_handle));

    // 4. Initialize controller
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);

    // 5. Custom command for ILI9486
    esp_lcd_panel_io_tx_param(io_h, 0x01, NULL, 0); // Soft Reset
    vTaskDelay(pdMS_TO_TICKS(120));

    esp_lcd_panel_io_tx_param(io_h, 0x11, NULL, 0); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));

    uint8_t fmt = 0x55;
    esp_lcd_panel_io_tx_param(io_h, 0x3A, &fmt, 1); // 16-bit RGB565
    
    uint8_t b6_data[] = {0x0A, 0xA2, 0x27}; 
    esp_lcd_panel_io_tx_param(io_h, 0xB6, b6_data, 3);

    uint8_t madctl = 0x08; // BGR order + Orientation //0x28
    esp_lcd_panel_io_tx_param(io_h, 0x36, &madctl, 1);
    
    uint8_t b7_data = 0x07; // Стандарт: 0x06. Попробуйте изменить на 0x04 или 0x07
    esp_lcd_panel_io_tx_param(io_h, 0xB7, &b7_data, 1);


    esp_lcd_panel_io_tx_param(io_h, 0x29, NULL, 0); // Display ON
    vTaskDelay(pdMS_TO_TICKS(20));

    // 6. ТЕСТ: Заливка полоски (DMA)
    uint16_t *black_buf = heap_caps_malloc(320 * 40 * 2, MALLOC_CAP_DMA);
    if (black_buf) 
    {
        for(int i=0; i<320*40; i++) black_buf[i] = 0xF01F; // Синий для теста (заметнее черного)
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 320, 40, black_buf);
        // Не удаляем буфер сразу, так как DMA может еще передавать данные
    }
    uint16_t *green_buf = heap_caps_malloc(320 * 40 * 2, MALLOC_CAP_DMA);
    if (green_buf) 
    {
        for(int i=0; i<320*40; i++) green_buf[i] = 0xFD0A; // Синий для теста (заметнее черного)
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 320, 40, green_buf);
        // Не удаляем буфер сразу, так как DMA может еще передавать данные
    }

    
    // 1. Конфигурация порта (создает задачу для LVGL)
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    // 2. Конфигурация дисплея для LVGL
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_h,       // Ваш хендл из esp_lcd_new_panel_io_i80
        .panel_handle = panel_handle, // Ваш хендл из esp_lcd_new_panel_st7789 (или другой)
        .buffer_size = LCD_H_RES * 40,// Размер буфера (например, 40 строк)
        .double_buffer = true,        // Включите, если есть PSRAM (убирает мерцание)
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        //.mipi_dsi = false,
        .flags = {
            .buff_dma = true,         // Обязательно для i8080
            .buff_spiram = false,     // true, если выделяете в PSRAM
        }
    };

    // 3. Добавление дисплея в порт
    lv_display_t * disp = lvgl_port_add_disp(&disp_cfg);

    // Это стандартный способ LVGL 9, работает везде
    lv_indev_t * enc_indev = lv_indev_create();            // Создаем устройство
    lv_indev_set_type(enc_indev, LV_INDEV_TYPE_ENCODER);   // Указываем, что это энкодер
    lv_indev_set_read_cb(enc_indev, encoder_read_cb);      // Назначаем ваш callback
    lv_indev_set_display(enc_indev, disp);                 // Привязываем к дисплею

    //lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
    //lv_display_set_render_mode(disp, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90); // Или 180, 270

    lvgl_port_lock(50);

    // 1. Создаем группу
    lv_group_t * g = lv_group_create();
    lv_group_set_default(g); // Теперь все новые кнопки будут автоматом падать в эту группу

    // 2. Привязываем энкодер к группе
    lv_indev_set_group(enc_indev, g);

    // 3. Создаем кнопки (они сами добавятся в группу 'g')
    lv_obj_t * btn1 = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn1, 100, 50);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -30); // По центру, смещение вверх
    // Создаем текст ВНУТРИ btn1
    lv_obj_t * label1 = lv_label_create(btn1); 
    lv_label_set_text(label1, "Start");      // Текст кнопки
    lv_obj_center(label1);                   // Центрируем текст в кнопке


    lv_obj_t * btn2 = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn2, 100, 50);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 30);  // По центру, смещение вниз на 30px

    // Создаем текст ВНУТРИ btn2
    lv_obj_t * label2 = lv_label_create(btn2);
    lv_label_set_text(label2, "Stop");
    lv_obj_center(label2);

    lvgl_port_unlock();

    lvgl_port_lock(50);

    // 1. Создаем дугу
    lv_obj_t * arc = lv_arc_create(lv_screen_active());
    lv_obj_set_size(arc, 150, 150);
    lv_arc_set_rotation(arc, 135);
    lv_arc_set_bg_angles(arc, 0, 270);
    lv_obj_center(arc);

    // 2. Скрываем стандартный серый круг ручки
    lv_obj_set_style_bg_opa(arc, 0, LV_PART_KNOB);      // Делаем фон ручки прозрачным
    lv_obj_set_style_shadow_width(arc, 0, LV_PART_KNOB); // Убираем тень

    // 3. Добавляем стрелку как текст внутри ручки
    lv_obj_t * label_arrow = lv_label_create(arc);
    // LV_PART_KNOB сам по себе не может содержать объекты, 
    // поэтому мы создаем label и привязываем его положение к ручке
    lv_label_set_text(label_arrow, LV_SYMBOL_UP); 
    lv_obj_set_style_text_color(label_arrow, lv_palette_main(LV_PALETTE_RED), 0);

    // 4. Чтобы стрелка двигалась за дугой, используем событие
    // В LVGL 9 это можно сделать через обновление координат в callback
    // Но проще всего добавить стиль "вращения" текста

    lvgl_port_unlock();
}



void update_ili9486()
{

}