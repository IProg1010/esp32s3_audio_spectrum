/*
Аналоговый вход:
AIN0 - GPIO1 - 1
Сигналы дисплея:
D0	GPIO 39	Шина данных (Bit 0)
D1	GPIO 40	Шина данных (Bit 1)
D2	GPIO 41	Шина данных (Bit 2)
D3	GPIO 42	Шина данных (Bit 3)
D4	GPIO 45	Шина данных (Bit 4)
D5	GPIO 46	Шина данных (Bit 5)
D6	GPIO 47	Шина данных (Bit 6)
D7	GPIO 48	Шина данных (Bit 7)
WR (Write Clock)	GPIO 4	Строб записи
RD (Read)	GPIO 5	Чтение (или подтяни к 3.3V, если не читаешь)
DC (RS)	GPIO 6	Команда / Данные
CS	GPIO 7	Выбор чипа
RST	GPIO 15	Сброс
*/

#include <stdio.h>
#include <math.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"

#include "dsps_fft2r.h"
#include "dsps_wind_hann.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2s_std.h"

#define FFT_SIZE 1024

// Назначаем пины согласно схеме
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

// Глобальные хендлы (чтобы были доступны везде)
esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_io_handle_t io_h = NULL; 

#define ADC_FRAME_SIZE  FFT_SIZE * 2 // 1024 семпла по 2 байта (тип Type1)
adc_continuous_handle_t adc_handle = NULL;

void init_adc_dma() {
    // 1. Создаем хендл непрерывного АЦП
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096, // Внутренний буфер драйвера
        .conv_frame_size = ADC_FRAME_SIZE, // Сколько данных ждем за один раз
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    // 2. Настраиваем параметры сканирования
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 44100, // Твоя четкая частота дискретизации
        .conv_mode = ADC_CONV_SINGLE_UNIT_1, // Только ADC1
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1, // Формат Type1 (12 бит данные + 4 бита канал)
    };

    adc_digi_pattern_config_t adc_pattern = {
        .atten = ADC_ATTEN_DB_12, // 0..3.1V (как у тебя было)
        .channel = ADC_CHANNEL_0, // GPIO 1
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    
    dig_cfg.pattern_num = 1;
    dig_cfg.adc_pattern = &adc_pattern;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
    
    // Стартуем АЦП
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

i2s_chan_handle_t tx_handle = NULL;

void init_i2s_dac() {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(192000), // Частота как у АЦП!
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_14,
            .ws = GPIO_NUM_12,
            .dout = GPIO_NUM_13,
            .din = I2S_GPIO_UNUSED,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
}

#define SINE_LUT_SIZE 256
int16_t sine_lut[SINE_LUT_SIZE];

void prepare_sine_table() {
    for (int i = 0; i < SINE_LUT_SIZE; i++) {
        // Генерируем значения от -32767 до 32767
        sine_lut[i] = (int16_t)(sinf(2.0f * M_PI * i / (SINE_LUT_SIZE/4)) * 32767);
    }
}

void play_iq_test_tone(i2s_chan_handle_t tx_handle) {
    int16_t i2s_buffer[512]; // Буфер для одной итерации (256 семплов стерео)
    uint32_t phase_i = 0;
    uint32_t phase_q = SINE_LUT_SIZE / 4; // Сдвиг 90 градусов (Косинус)
    
    // Шаг фазы для 1 кГц при частоте 44100 Гц:
    // step = (Freq_target / Freq_sample) * LUT_SIZE
    float step = (1000.0f / 44000.0f) * SINE_LUT_SIZE;
    float current_phase = 0;

    while (1) {
        for (int n = 0; n < 256; n++) {
            int idx_i = (int)current_phase % SINE_LUT_SIZE;
            int idx_q = (int)(current_phase + (SINE_LUT_SIZE / 4)) % SINE_LUT_SIZE;

            i2s_buffer[n * 2]     = sine_lut[idx_i]; // Канал I (Left)
            i2s_buffer[n * 2 + 1] = sine_lut[idx_q]; // Канал Q (Right)
            
            current_phase += step;
            if (current_phase >= SINE_LUT_SIZE) current_phase -= SINE_LUT_SIZE;
        }

        size_t written;
        i2s_channel_write(tx_handle, i2s_buffer, sizeof(i2s_buffer), &written, portMAX_DELAY);
        
        // Маленькая пауза, чтобы не блокировать watchdog, если это не в отдельной задаче
        //vTaskDelay(1); 
    }
}


void init_ili9486_parallel() {
    // 1. Настройка шины I80
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = LCD_DC,
        .wr_gpio_num = LCD_WR,
        .data_gpio_nums = { LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7 },
        .bus_width = 8,
        .max_transfer_bytes = 480 * 40 * 2, // Уменьшил для стабильности DMA
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    // 2. Настройка IO (Интерфейс)
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

    // 3. СОЗДАНИЕ ОБЪЕКТА ПАНЕЛИ (Этого не хватало!)
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    // Используем ST7789 как базовый драйвер для ILI9486
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_h, &panel_config, &panel_handle));

    // 4. Инициализация контроллера
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);

    // 5. Кастомные команды для ILI9486 (выход из комы)
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
    uint16_t *black_buf = heap_caps_malloc(480 * 40 * 2, MALLOC_CAP_DMA);
    if (black_buf) {
        for(int i=0; i<480*40; i++) black_buf[i] = 0x001F; // Синий для теста (заметнее черного)
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 40, black_buf);
        // Не удаляем буфер сразу, так как DMA может еще передавать данные
    }
}
float fft_input[FFT_SIZE * 2]; // Массив для FFT (Re, Im)
float window[FFT_SIZE];
uint16_t adc_raw[FFT_SIZE];

void draw_bar_in_canvas(uint16_t *buffer, int x, int h, int w) {
    // Ограничиваем высоту (наш холст 100 пикселей)
    if (h > 100) h = 100;
    
    // Рисуем столбик снизу вверх
    for (int y = 0; y < 100; y++) {
        for (int dx = 0; dx < w; dx++) {
            int pixel_idx = y * 480 + (x + dx);
            if (pixel_idx < 480 * 100) {
                // Если координата Y выше столбика — рисуем черный, если ниже — зеленый
                buffer[pixel_idx] = (y >= (100 - h)) ? 0x07E0 : 0x0000;
            }
        }
    }
}
void app_main(void) {
    /*// 1. Инициализация ADC на GPIO 1
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
*/
    prepare_sine_table();
    init_i2s_dac(); // функция инициализации I2S
    
    printf("Запуск I/Q теста 1 кГц...\n");
    // 2. Инициализация FFT (библиотека dsp)
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) { ESP_LOGE("DSP", "Ошибка FFT!"); return; }
    dsps_wind_hann_f32(window, FFT_SIZE);
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LCD_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    /*gpio_config(&io_conf);

    gpio_set_level(LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(200));*/
    init_adc_dma();
    init_ili9486_parallel();

    uint16_t *black_buf = heap_caps_malloc(480 * 100 * sizeof(uint16_t), MALLOC_CAP_DMA);
    uint16_t *green_buf = heap_caps_malloc(8 * 100 * sizeof(uint16_t), MALLOC_CAP_DMA); // Столбик шириной 8 пикселей

    // Заполняем их цветами один раз
    for(int i=0; i < 480*100; i++) black_buf[i] = 0x0000; // Черный
    for(int i=0; i < 8*100; i++) green_buf[i] = 0x07E0;   // Зеленый

    uint16_t *canvas = heap_caps_malloc(480 * 100 * 2, MALLOC_CAP_DMA);
        // Создаем один столбик шириной 10 и высотой 100 пикселей
    uint16_t *red_bar = heap_caps_malloc(10 * 100 * sizeof(uint16_t), MALLOC_CAP_DMA);
    for (int i = 0; i < 10 * 100; i++) {
        red_bar[i] = 0xF800; // Ярко-красный (RGB565)
    }

    // Один вертикальный буфер шириной 10 и высотой 150
    uint16_t *bar_data = heap_caps_malloc(10 * 150 * sizeof(uint16_t), MALLOC_CAP_DMA);
    uint8_t result[ADC_FRAME_SIZE];
    uint32_t ret_num = 0;
    while (1) {
    play_iq_test_tone(tx_handle);
        // Читаем данные (для простоты пока в цикле, позже переведем на DMA)
        /*for (int i = 0; i < FFT_SIZE; i++) {
            int val;
            adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &val);
            fft_input[i * 2] = (float)val * window[i]; // Окно Ханна
            fft_input[i * 2 + 1] = 0;                 // Мнимая часть = 0
        }

        // Выполняем FFT
        dsps_fft2r_fc32(fft_input, FFT_SIZE);
        dsps_bit_rev_fc32(fft_input, FFT_SIZE);

        // Рисуем простейший спектр в монитор (первые 40 бинов)*/
        /*printf("\x1b[H"); // Очистка экрана терминала
        for (int i = 0; i < 40; i++) {
            float amplitude = sqrtf(fft_input[i*2]*fft_input[i*2] + fft_input[i*2+1]*fft_input[i*2+1]) / 100;
            printf("%2d: ", i);
            for (int j = 0; j < (int)amplitude; j++) printf("#");
            printf("\n");
        }*/
       // Очистка старого спектра (черная полоса)
        /*esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 100, black_buf); 

        for (int i = 0; i < 60; i++) { // Рисуем 60 столбиков
            float amplitude = sqrtf(fft_input[i*2]*fft_input[i*2] + fft_input[i*2+1]*fft_input[i*2+1]) / 50;
            int bar_height = (int)amplitude;
            if (bar_height > 100) bar_height = 100; // Ограничение высоты

            // Рисуем столбик шириной 6 пикселей (с зазором 2 пикселя)
            // Используем тот же green_buf, но с нужной высотой
            esp_lcd_panel_draw_bitmap(panel_handle, i * 8, 100 - bar_height, i * 8 + 6, 100, green_buf);
        }*/
       /*memset(canvas, 0, 480 * 100 * 2); // Очистка памяти (черный)
        for (int i = 0; i < 40; i++) {
            float mag = sqrtf(fft_input[i*2]*fft_input[i*2] + fft_input[i*2+1]*fft_input[i*2+1]) / 20;
            int h = (int)mag;
            // Рисуем i-й столбик: x = i*12, высота = h, ширина = 10
            draw_bar_in_canvas(canvas, i * 12, h, 10);
        }

    // 4. ОДНИМ МАХОМ отправляем всё на дисплей (никакого мерцания!)
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 100, canvas);

        // Рисуем 30 столбиков в разных позициях X
        for (int i = 0; i < 30; i++) {
            int x_pos = i * 15; // Шаг 15 пикселей
            int height = 50 + (i * 2); // Ступенчатая высота для теста
            
            // Рисуем красный столбик (высота фиксированная 100 для простоты буфера)
            esp_lcd_panel_draw_bitmap(panel_handle, x_pos, 0, x_pos + 10, 100, red_bar);
        }
        
        printf("Красные столбики отрисованы\n");*/
        // Ждем готовую порцию данных от DMA (таймаут 100мс)
        esp_err_t err = adc_continuous_read(adc_handle, result, ADC_FRAME_SIZE, &ret_num, 100);
        
        if (err == ESP_OK) {
            float sum = 0;
            int count = ret_num / SOC_ADC_DIGI_RESULT_BYTES;

            // 1. Извлекаем данные и считаем сумму для среднего
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                uint32_t val = p->type2.data; // Используем Type2
                
                int idx = i / SOC_ADC_DIGI_RESULT_BYTES;
                if (idx < FFT_SIZE) {
                    adc_raw[idx] = (float)val;
                    sum += adc_raw[idx];
                }
            }

            // 2. Считаем среднее (постоянку)
            float mean = sum / count;

            // 3. Подготовка к FFT: вычитаем среднее и накладываем окно
            for (int i = 0; i < FFT_SIZE; i++) {
                // (Значение - Постоянка) * Окно Ханна
                fft_input[i * 2] = (adc_raw[i] - mean) * window[i]; 
                fft_input[i * 2 + 1] = 0;
            }

            // 4. Выполняем FFT
            dsps_fft2r_fc32(fft_input, FFT_SIZE);
            dsps_bit_rev_fc32(fft_input, FFT_SIZE);
        }

        // Рассчитываем амплитуду (подбери делитель под свой микрофон)
        for (int i = 0; i < 40; i++) {
            // Рассчитываем амплитуду (подбери делитель под свой микрофон)
            float mag = sqrtf(fft_input[i*2]*fft_input[i*2] + fft_input[i*2+1]*fft_input[i*2+1]) / 20;
            int h = (int)mag-35;
            if (h > 150) h = 150; // Потолок

            // Заполняем буфер одного столбика: верх черный, низ цветной
            for (int y = 0; y < 150; y++) {
                uint16_t color = (y < (150 - h)) ? 0x0000 : 0xF800; // Черный или Красный
                for (int x = 0; x < 10; x++) {
                    bar_data[y * 10 + x] = color;
                }
            }

            // Рисуем столбик целиком (он сам себя стирает сверху!)
            // i*12 — это позиция X с зазором 2 пикселя
            esp_lcd_panel_draw_bitmap(panel_handle, i*12, 0, i*12 + 10, 150, bar_data);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}