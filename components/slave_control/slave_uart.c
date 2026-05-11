#include "slave_uart.h"
#include "wchisp.h"
#include "wch-isp.h"
#include "dev_iface.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "input_manager.h"

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
QueueHandle_t queue_enc1;
QueueHandle_t queue_enc2;

extern const uint8_t bin_start[] asm("_binary_flash_slave_bin_start");
extern const uint8_t bin_end[]   asm("_binary_flash_slave_bin_end");
size_t bin_size;

device_conf dev_funct;

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
    bin_size = bin_end - bin_start;
    printf("Найдена прошивка подчиненного контроллера: %d байт\n", bin_size);
    //flash_firmware((const uint8_t*) bin_start, bin_size);
}

void init_uart_for_isp() 
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


void init_uart_for_proto() 
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

int writeUart(uint8_t* buff, uint16_t size)
{
    uart_flush_input(UART_PORT_NUM);
    uart_write_bytes(UART_PORT_NUM, buff, size);
    //uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100));
    return size;
}

int readUart(uint8_t* buff, uint16_t size, int* get_cnt)
{
    *get_cnt = uart_read_bytes(UART_PORT_NUM, buff, size, pdMS_TO_TICKS(240));
    return *get_cnt;
}

void initSlave()
{
    dev_funct.write = writeUart;
    dev_funct.read = readUart;
    set_function(&dev_funct);
    find_slave_controller_firmware();
    vTaskDelay(pdMS_TO_TICKS(1000));    // Пауза на запуск*/

    init_uart_for_isp();
    vTaskDelay(pdMS_TO_TICKS(1000));    // Пауза на запуск
    slave_control_enter_bootloader();
    vTaskDelay(pdMS_TO_TICKS(500));    // Пауза на запуск
    slave_control_enter_app();
    vTaskDelay(pdMS_TO_TICKS(500));    // Пауза на запуск
    slave_control_enter_bootloader();

    vTaskDelay(pdMS_TO_TICKS(1000));    // Пауза на запуск
    wch_identify_chip(UART_PORT_NUM);
    wch_flash_erase(UART_PORT_NUM, 100); //  
    //isp_cmd_identify_glob();
    program_wchisp_algo();
    cmd_write_flash_bin((const uint8_t*) bin_start, bin_size);
    printf("program ok\n");
    vTaskDelay(pdMS_TO_TICKS(500));    // Пауза на запуск

    uart_driver_delete(UART_PORT_NUM);

    slave_control_enter_app();
    vTaskDelay(pdMS_TO_TICKS(500)); 

    init_uart_for_proto();

    queue_enc1 = get_input_queue_enc1();
    queue_enc2 = get_input_queue_enc2();
}

void writeToSlave()
{

}

void readFromSlave()
{
    uint8_t buff[20];
    size_t size = 20;
    int len = uart_read_bytes(UART_PORT_NUM, buff, size, pdMS_TO_TICKS(240));
    printf("data:");
    for(int i = 0; i < len; i++)
    {
        printf("%c", buff[i]);
    }
    printf("\n");

    if(len != 0)
    {
        input_event_t event;
        
        // Твоя логика из начала разговора:
        if(buff[0] == 'v') 
            if(buff[6] == '-') event = ENC_DOWN;
            else event = ENC_UP;
        else if(buff[0] == 'p') event = ENC_PUSH;
        else if(buff[0] == 'r') event = ENC_RELEASE;
        
        xQueueSend(queue_enc1, &event, 0);
    }
}
