#include <stdint.h>
#include <string.h>
#include "driver/uart.h"

/* WCH ISP Protocol Constants */
#define WCH_ISP_HEADER_1     0x57
#define WCH_ISP_HEADER_2     0xAB
#define WCH_ISP_MAX_PAYLOAD 60

/*
 *  All readable and writable registers.
 *  - `RDPR`: Read Protection
 *  - `USER`: User Config Byte (normally in Register Map datasheet)
 *  - `WPR`:  Write Protection Mask, 1=unprotected, 0=protected
 *
 *  | BYTE0  | BYTE1  | BYTE2  | BYTE3  |
 *  |--------|--------|--------|--------|
 *  | RDPR   | nRDPR  | USER   | nUSER  |
 *  | DATA0  | nDATA0 | DATA1  | nDATA1 |
 *  | WPR0   | WPR1   | WPR2   | WPR3   |
 */
#define CFG_MASK_USERCONF 0x07 /* 12 bytes, see table above */
#define CFG_MASK_BTVER 0x08 /* Bootloader version, in the format of `[0x00, major, minor, 0x00]` */
#define CFG_MASK_UID 0x10 /* Device Unique ID */
#define CFG_MASK_ALL 0x1f /* All mask bits of CFGs */


#define WCH_RESPONSE_TIMEOUT_MS  500

/* Command Keys from protocol.rs */
typedef enum {
    IDENTIFY = 0xa1,
    ISP_END = 0xa2,
    ISP_KEY = 0xa3,
    ERASE = 0xa4,
    PROGRAM = 0xa5,
    VERIFY = 0xa6,
    READ_CONFIG = 0xa7,
    WRITE_CONFIG = 0xa8,
    DATA_ERASE = 0xa9,
    DATA_PROGRAM = 0xaa,
    DATA_READ = 0xab,
    WRITE_OTP = 0xc3,
    READ_OTP = 0xc4,
    SET_BAUD = 0xc5,
} wch_cmd_t;

/* Packet Structure */
#pragma pack(push, 1)
typedef struct {
    uint8_t header;
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[WCH_ISP_MAX_PAYLOAD];
} wch_packet_t;
#pragma pack(pop)

/**
 * @brief Calculates checksum as sum of all bytes modulo 256
 * @param packet Pointer to the packet structure
 * @param data_len Length of the payload only
 * @return uint8_t checksum
 */
static uint8_t wch_calculate_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload) 
{
    // 1. Берем команду
    uint32_t sum = cmd;
    
    // 2. Добавляем оба байта длины (так как в Rust len: u16)
    sum += (uint8_t)(len & 0xFF);         // Младший байт
    sum += (uint8_t)((len >> 8) & 0xFF);  // Старший байт
    
    // 3. Добавляем все байты полезной нагрузки
    if (payload != NULL) {
        for (uint16_t i = 0; i < len; i++) {
            sum += payload[i];
        }
    }
    
    // Возвращаем младший байт суммы
    return (uint8_t)(sum & 0xFF);
}

uint8_t calculate_crc(const uint8_t *data, size_t len) 
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) 
    {
        crc += data[i]; // Переполнение произойдет автоматически (wrapping)
    }
    return crc;
}

/**
 * @brief Чтение и проверка ответа от чипа
 */
esp_err_t wch_read_response(uart_port_t uart_num, uint8_t *cmd_out, uint8_t *payload, uint8_t *len_out) 
{
    uint8_t header[3]; // [Header, Cmd, Len]
    
    // 1. Читаем заголовок (первые 3 байта)
    int rx_len = uart_read_bytes(uart_num, header, 3, pdMS_TO_TICKS(WCH_RESPONSE_TIMEOUT_MS));
    if (rx_len < 3 || header[0] != WCH_ISP_HEADER_1) 
    {
        return ESP_FAIL;
    }

    uint8_t cmd = header[1];
    uint8_t len = header[2];
    printf("cmd = %02x, len = %02x", cmd, len);

    // 2. Читаем полезную нагрузку + 1 байт чексуммы
    int remaining = len + 1;
    uint8_t temp_buf[WCH_ISP_MAX_PAYLOAD + 1];
    
    rx_len = uart_read_bytes(uart_num, temp_buf, remaining, pdMS_TO_TICKS(WCH_RESPONSE_TIMEOUT_MS));
    if (rx_len < remaining) 
    {
        return ESP_ERR_TIMEOUT;
    }

    // 3. Проверка контрольной суммы (checksum находится в конце temp_buf)
    uint8_t received_checksum = temp_buf[len];
    if (received_checksum != wch_calculate_checksum(cmd, len, temp_buf)) 
    {
        return ESP_ERR_INVALID_CRC;
    }

    // 4. Копируем результат
    if (payload) memcpy(payload, temp_buf, len);
    if (cmd_out) *cmd_out = cmd;
    if (len_out) *len_out = len;

    return ESP_OK;
}

/**
 * @brief Sends a formatted command to WCH chip via UART
 * @param uart_num ESP32 UART port number
 * @param cmd Command key (wch_cmd_t)
 * @param payload Pointer to data to send
 * @param len Length of data
 */
void wch_send_command(uart_port_t uart_num, uint8_t cmd, const uint8_t *payload, uint8_t len) 
{
    uint8_t buffer[WCH_ISP_MAX_PAYLOAD + 10];
    
    buffer[0] = WCH_ISP_HEADER_1; // 0x57 (обычно для WCH)
    buffer[1] = cmd;
    
    // Записываем длину как uint16_t (Little Endian)
    buffer[2] = (uint8_t)(len & 0xFF);
    buffer[3] = (uint8_t)((len >> 8) & 0xFF);
    
    if (payload != NULL && len > 0) {
        memcpy(&buffer[4], payload, len);
    }
    
    // Чексумма в этом протоколе обычно считается от CMD + LEN + PAYLOAD
    buffer[4 + len] = wch_calculate_checksum(cmd, len, payload);
    
    // Итого отправляем: Header(1) + CMD(1) + Len(2) + Payload(n) + Checksum(1)
    uart_write_bytes(uart_num, (const char *)buffer, len + 5);
}

/**
 * @brief Example: Identify/Initialize WCH Chip
 */
/**
 * @brief Идентификация чипа с получением ID
 */
void wch_identify_chip(uart_port_t uart_num) 
{
    /*// Размер полезной нагрузки согласно Rust: 0x12 (18 байт)
    uint8_t payload[18]; 
    memset(payload, 0, sizeof(payload));

    payload[0] = 0x00;//device_id;
    payload[1] = 0x11;//device_type;
    
    // Копируем магическую строку "MCU ISP & WCH.CN" (16 байт)
    memcpy(&payload[2], "MCU ISP & WCH.CN", 16);

    // Отправляем: Header + CMD + Len(0x12) + Payload + Checksum
    wch_send_command(uart_num, WCH_CMD_IDENTIFY, payload, 18);
    //uint8_t payload_send[] = {0x00}; 
    //wch_send_command(uart_num, WCH_CMD_IDENTIFY, payload_send, sizeof(payload_send));

    // Буфер для приема ID (обычно 2-6 байт в зависимости от модели)
    uint8_t response_payload[WCH_ISP_MAX_PAYLOAD];
    uint8_t resp_cmd, resp_len;

    if (wch_read_response(uart_num, &resp_cmd, response_payload, &resp_len) == ESP_OK)
    {
        // Выводим полученный ID в лог
        printf("Chip Identified! ID: ");
        for(int i = 0; i < resp_len; i++) 
        {
            printf("%02X ", response_payload[i]);
        }
        printf("\n");
    } 
    else 
    {
        printf("Failed to get response from WCH chip\n");
    }*/
     // Формируем payload как в Rust: [ID, Type, "MCU ISP & WCH.CN"]

    //uint8_t buffer[1] = {0x55};
    //uart_write_bytes(uart_num, (const char *)buffer, 1);
    
    //vTaskDelay(pdMS_TO_TICKS(1000));    // Пауза на запуск

    
    uint8_t buffer[1] = { 0x55 };
    uart_write_bytes(uart_num, buffer, 1);
    // Очищаем буфер перед отправкой
    uart_flush_input(uart_num);
    
    // Отправляем (длина 18 = 0x12)
    //wch_send_command(uart_num, 0xBB, payload, 18);
    uint8_t payload[24];

    payload[0] = WCH_ISP_HEADER_1;
    payload[1] = WCH_ISP_HEADER_2; 

    uint16_t len_crc = 0;
    
    payload[2] = IDENTIFY; //
    len_crc++;
    // Записываем длину как uint16_t (Little Endian)
    uint16_t len = 0x12;
    payload[3] = (uint8_t)(len & 0xFF);
    len_crc++;
    payload[4] = (uint8_t)((len >> 8) & 0xFF);
    len_crc++;
    payload[5] = 0x00; // device_id
    len_crc++;
    payload[6] = 0x00; // device_type (0x11 для V203)
    len_crc++;
    memcpy(&payload[7], "MCU ISP & WCH.CN", 16);

    len_crc += 16;

    payload[23] = calculate_crc(&payload[2], len_crc);

    // Итого отправляем: Header(1) + CMD(1) + Len(2) + Payload(n) + Checksum(1)
    for(int i = 0; i < 24; i++)
    {
        printf("%02X", payload[i]);
    }
    uart_write_bytes(uart_num, payload, 24);


    // Читаем ответ
    uint8_t rx_buf[64];
    // Ответ обычно: Header + CMD + Len(2b) + Data + Checksum

    int rx_len = uart_read_bytes(uart_num, rx_buf, 64, pdMS_TO_TICKS(WCH_RESPONSE_TIMEOUT_MS));

    printf("\n");
    printf("rx_buf[0] = 0x%02X\n", rx_buf[0]);
    if (rx_len >= 6) {
        uint8_t crc = calculate_crc(&rx_buf[2], rx_len-3);
        if(crc == rx_buf[rx_len-1])
        {
            printf("ok\n");
        }
        // В ответе на Identify чип присылает свои реальные данные
        uint8_t chip_id = rx_buf[rx_len-3];   // Первый байт данных
        uint8_t chip_type = rx_buf[rx_len-2]; // Второй байт данных
        for(int i =0; i < rx_len; i++)
        {
            printf("%02X", rx_buf[i]);    
        }
        printf("\n");    
        printf("Чип определен! ID: %02d, Type: %02d\n", chip_id, chip_type);
    } else {
        printf("Ошибка: ответ не получен или слишком короткий len = 0x%02X\n", rx_len);
    }
}

/**
 * @brief Example: Program Data (Flash)
 * @param addr 32-bit start address
 */
void wch_flash_write(uart_port_t uart_num, uint32_t addr, uint8_t *data, uint8_t data_len) 
{
    
}

void wch_flash_erase(uart_port_t uart_num, uint32_t sector_count) 
{
    uart_flush_input(uart_num);
    
    // Отправляем (длина 18 = 0x12)
    //wch_send_command(uart_num, 0xBB, payload, 18);
    uint8_t payload[24];

    payload[0] = WCH_ISP_HEADER_1;
    payload[1] = WCH_ISP_HEADER_2; 

    uint16_t len_crc = 0;
    
    payload[2] = ERASE; //
    len_crc++;
    // Записываем длину как uint16_t (Little Endian)
    uint16_t len = 0x04;
    payload[3] = (uint8_t)(len & 0xFF);
    len_crc++;
    payload[4] = (uint8_t)((len >> 8) & 0xFF);
    len_crc++;
    payload[5] = (uint8_t)(sector_count & 0xFF);
    len_crc++;
    payload[6] = (uint8_t)((sector_count >> 8) & 0xFF);
    len_crc++;
    payload[7] = (uint8_t)((sector_count >> 16) & 0xFF); // device_id
    len_crc++;
    payload[8] = (uint8_t)((sector_count >> 24) & 0xFF); // device_type (0x11 для V203)
    len_crc++;

    payload[9] = calculate_crc(&payload[2], len_crc);

    // Итого отправляем: Header(1) + CMD(1) + Len(2) + Payload(n) + Checksum(1)
    for(int i = 0; i < 10; i++)
    {
        printf("%02X", payload[i]);
    }
    uart_write_bytes(uart_num, payload, 10);
    uart_wait_tx_done(uart_num, pdMS_TO_TICKS(1500));


    // Читаем ответ
    uint8_t rx_buf[64];
    // Ответ обычно: Header + CMD + Len(2b) + Data + Checksum

    int rx_len = uart_read_bytes(uart_num, rx_buf, 64, pdMS_TO_TICKS(1500));

    printf("\n");
    printf("rx_buf[0] = 0x%02X\n", rx_buf[0]);
    for(int i =0; i < rx_len; i++)
    {
        printf("%02X", rx_buf[i]);    
    }
    if (rx_len >= 6) 
    {
        uint8_t crc = calculate_crc(&rx_buf[2], rx_len-3);
        if(crc == rx_buf[rx_len-1])
        {
            printf("ok\n");
        }
        // В ответе на Identify чип присылает свои реальные данные
        for(int i =0; i < rx_len; i++)
        {
            printf("%02X", rx_buf[i]);    
        }
        printf("\n");    
        printf("Сектор очишен\n");
    } 
    else 
    {
        printf("Ошибка: ответ не получен или неправльный len = 0x%02X\n", rx_len);
    }
}

void wch_flash_read(uart_port_t uart_num, uint32_t sector_count) 
{

}



