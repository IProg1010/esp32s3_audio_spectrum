#include "slave_uart.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT       UART_NUM_1
#define CH32_BOOT0_PIN  GPIO_NUM_5
#define CH32_RST_PIN    GPIO_NUM_18

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE (1024)

static const char *TAG = "WCH_ISP";

// Command WCH ISP
#define CMD_SYNC       0x55
#define CMD_ID         0x11
#define CMD_ERASE      0x16
#define CMD_WRITE      0x18
#define CMD_EXIT       0x12

void slave_control_enter_bootloader();
void flash_slave_controller(); 
esp_err_t flash_firmware(const uint8_t* data, size_t size);


extern const uint8_t bin_start[] asm("_binary_flash_slave_bin_start");
extern const uint8_t bin_end[]   asm("_binary_flash_slave_bin_end");


void slave_control_enter_bootloader()
{
    gpio_set_direction(CH32_BOOT0_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(CH32_RST_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(CH32_BOOT0_PIN, 1); // BOOT0 HIGH
    gpio_set_level(CH32_RST_PIN, 0);   // RESET LOW
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(CH32_RST_PIN, 1);   // RESET HIGH
    vTaskDelay(pdMS_TO_TICKS(100));    // Пауза на запуск
}

void flash_slave_controller() 
{
    // 1. Calculate bootloader size
    size_t bin_size = bin_end - bin_start;
    
    printf("Найдена прошивка подчиненного контроллера: %d байт\n", bin_size);


    // 2. Transfer CH32 to mode uart bootloader
    slave_control_enter_bootloader(); 

    // 3. Send firmware bytes from uart
    // Используйте bin_start как обычный массив:
    //uart_write_bytes(UART_NUM_1, (const char*)bin_start, bin_size);
    flash_firmware((const uint8_t*) bin_start, bin_size);
}

esp_err_t flash_firmware(const uint8_t* data, size_t size) 
{
    uint8_t buf[256];
    
    // 1. Synchrinization (Auto-baud)
    buf[0] = CMD_SYNC;
    uart_write_bytes(UART_PORT, (char*)buf, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 2. Identification command
    uint8_t cmd_id[] = {0x57, 0xAB, CMD_ID, 0x00};
    uart_write_bytes(UART_PORT, (char*)cmd_id, 4);
    // Тут в идеале ждать ответ с ID чипа

    // 3. Erase flash command
    uint8_t cmd_erase[] = {0x57, 0xAB, CMD_ERASE, 0x00};
    uart_write_bytes(UART_PORT, (char*)cmd_erase, 4);
    vTaskDelay(pdMS_TO_TICKS(500)); // Ждем завершения стирания

    // 4. Запись данными (блоками по 64 байта)
    size_t offset = 0;
    while (offset < size) {
        size_t chunk = (size - offset > 64) ? 64 : (size - offset);
        
        // Заголовок команды записи: 0x57 0xAB, CMD, Len, Address(4 bytes)...
        // Для V203 адрес обычно инкрементируется автоматически или передается явно
        // Ниже упрощенная структура пакета
        buf[0] = 0x57; buf[1] = 0xAB;
        buf[2] = CMD_WRITE;
        buf[3] = (uint8_t)chunk; 
        memcpy(&buf[4], &data[offset], chunk);

        uart_write_bytes(UART_PORT, (char*)buf, chunk + 4);
        
        // Обязательно ждем подтверждения от чипа (обычно 0x00 или статус)
        // uart_read_bytes(UART_PORT, buf, 1, pdMS_TO_TICKS(100));

        offset += chunk;
        ESP_LOGI(TAG, "Прошито %d/%d", offset, size);
    }

    return ESP_OK;
}

void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,            // Стандарт для WCH ISP
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Установка параметров
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    // Установка пинов (TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Установка драйвера (выделяем буфер под прием данных)
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

void initSlave()
{

}

void writeToSlave()
{

}

void readFromSlave()
{

}
