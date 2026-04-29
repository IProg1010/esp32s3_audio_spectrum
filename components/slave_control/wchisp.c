#include <stdint.h>
#include <string.h>
#include "driver/uart.h"

/* WCH ISP Protocol Constants */
#define WCH_ISP_HEADER      0x55
#define WCH_ISP_MAX_PAYLOAD 60

/* Command Keys from protocol.rs */
typedef enum {
    WCH_CMD_IDENTIFY = 0x11,
    WCH_CMD_ISP_KEY  = 0x11, // Same as Identify in some contexts
    WCH_CMD_ERASE    = 0x12,
    WCH_CMD_PROGRAM  = 0x13,
    WCH_CMD_VERIFY   = 0x14,
    WCH_CMD_END      = 0x0D,
    WCH_CMD_RESET    = 0x15
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
    uint32_t sum = WCH_ISP_HEADER + cmd + len;
    for (int i = 0; i < len; i++) {
        sum += payload[i];
    }
    return (uint8_t)(sum & 0xFF);
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
    uint8_t buffer[WCH_ISP_MAX_PAYLOAD + 4];
    
    buffer[0] = WCH_ISP_HEADER;
    buffer[1] = cmd;
    buffer[2] = len;
    
    if (payload != NULL && len > 0) {
        memcpy(&buffer[3], payload, len);
    }
    
    // Checksum is placed right after payload
    buffer[3 + len] = wch_calculate_checksum(cmd, len, payload);
    
    uart_write_bytes(uart_num, (const char *)buffer, len + 4);
}

/**
 * @brief Example: Identify/Initialize WCH Chip
 */
void wch_identify_chip(uart_port_t uart_num) 
{
    // According to protocol.rs, identify payload is often [0x00] or specific device code
    uint8_t payload[] = {0x00}; 
    wch_send_command(uart_num, WCH_CMD_IDENTIFY, payload, sizeof(payload));
}

/**
 * @brief Example: Program Data (Flash)
 * @param addr 32-bit start address
 */
void wch_program_data(uart_port_t uart_num, uint32_t addr, uint8_t *data, uint8_t data_len) 
{
    uint8_t payload[WCH_ISP_MAX_PAYLOAD];
    
    // First 4 bytes usually represent the address (Little Endian)
    payload[0] = addr & 0xFF;
    payload[1] = (addr >> 8) & 0xFF;
    payload[2] = (addr >> 16) & 0xFF;
    payload[3] = (addr >> 24) & 0xFF;
    
    // Copy flash data after address
    memcpy(&payload[4], data, data_len);
    
    wch_send_command(uart_num, WCH_CMD_PROGRAM, payload, data_len + 4);
}