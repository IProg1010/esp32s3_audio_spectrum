#include "pcm5102.h"

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_std.h"


void initDAC()
{

}

void writeBuffer(uint16_t* data, uint16_t length)
{

}

void writezeroDAC()
{
    
}


i2s_chan_handle_t tx_handle = NULL;

void init_i2s_dac() 
{
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

void prepare_sine_table() 
{
    for (int i = 0; i < SINE_LUT_SIZE; i++) {
        // Генерируем значения от -32767 до 32767
        sine_lut[i] = (int16_t)(sinf(2.0f * M_PI * i / (SINE_LUT_SIZE/4)) * 32767);
    }
}

void play_iq_test_tone(i2s_chan_handle_t tx_handle) 
{
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