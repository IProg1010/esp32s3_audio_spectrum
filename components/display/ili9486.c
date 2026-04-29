
#include "ili9486.h"
#include "display.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

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


void init_ili9486();

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
        .pclk_hz = 2 * 1000 * 1000, 
        .trans_queue_depth = 10,
        .dc_levels = { .dc_idle_level = 0, .dc_cmd_level = 0, .dc_dummy_level = 0, .dc_data_level = 1 },
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = { .pclk_idle_low = 0 }, // Вместо pclk_active_neg
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
    
    uint8_t madctl = 0x28; // BGR order + Orientation
    esp_lcd_panel_io_tx_param(io_h, 0x36, &madctl, 1);

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
}

void update_ili9486()
{

}