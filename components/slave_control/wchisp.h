#ifndef WCH_ISP_H
#define WCH_ISP_H

#include "driver/uart.h"

//static uint8_t wch_calculate_checksum(uint8_t cmd, uint8_t len, const uint8_t *payload);

/**
 * @brief Sends a formatted command to WCH chip via UART
 * @param uart_num ESP32 UART port number
 * @param cmd Command key (wch_cmd_t)
 * @param payload Pointer to data to send
 * @param len Length of data
 */
void wch_send_command(uart_port_t uart_num, uint8_t cmd, const uint8_t *payload, uint8_t len);

/**
 * @brief Example: Identify/Initialize WCH Chip
 */
void wch_identify_chip(uart_port_t uart_num);
/**
 * @brief Example: Program Data (Flash)
 * @param addr 32-bit start address
 */
void wch_program_data(uart_port_t uart_num, uint32_t addr, uint8_t *data, uint8_t data_len);

#endif