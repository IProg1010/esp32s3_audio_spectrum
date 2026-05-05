#include "slave_uart.h"
#include "wchisp.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define CH32_BOOT0_PIN  GPIO_NUM_10
#define CH32_RST_PIN    GPIO_NUM_9

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define UART_PORT_NUM UART_NUM_2
#define BUF_SIZE (1024)

static const char *TAG = "WCH_ISP";

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
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(CH32_RST_PIN, 1);   // RESET HIGH
    vTaskDelay(pdMS_TO_TICKS(100));    // Пауза на запуск
}

void slave_control_enter_app()
{
    gpio_set_level(CH32_BOOT0_PIN, 0); // BOOT0 HIGH
    gpio_set_level(CH32_RST_PIN, 0);   // RESET LOW
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(CH32_RST_PIN, 1);   // RESET HIGH
    vTaskDelay(pdMS_TO_TICKS(100));    // Пауза на запуск
}

void find_slave_controller_firmware() 
{
    // 1. Calculate bootloader size
    size_t bin_size = bin_end - bin_start;
    printf("Найдена прошивка подчиненного контроллера: %d байт\n", bin_size);
    //flash_firmware((const uint8_t*) bin_start, bin_size);
}

void init_uart() 
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,            // Стандарт для WCH ISP
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Установка пинов (TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Установка параметров
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));


    // Установка драйвера (выделяем буфер под прием данных)
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

void flashSlave()
{

}

void initSlave()
{
    find_slave_controller_firmware();
    vTaskDelay(pdMS_TO_TICKS(1000));    // Пауза на запуск*/

    init_uart();
    vTaskDelay(pdMS_TO_TICKS(1000));    // Пауза на запуск
    slave_control_enter_bootloader();
    vTaskDelay(pdMS_TO_TICKS(500));    // Пауза на запуск
    slave_control_enter_app();
    vTaskDelay(pdMS_TO_TICKS(500));    // Пауза на запуск
    slave_control_enter_bootloader();

    vTaskDelay(pdMS_TO_TICKS(1000));    // Пауза на запуск
    wch_identify_chip(UART_PORT_NUM);
    wch_flash_erase(UART_PORT_NUM, 100); //  
}

void writeToSlave()
{

}

void readFromSlave()
{

}
