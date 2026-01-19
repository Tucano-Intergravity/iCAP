/*
 * UARTComm.h
 *
 *  Created on: Jan 6, 2026
 *      Author: USER
 */

#ifndef ICAP_INC_UARTCOMM_H_
#define ICAP_INC_UARTCOMM_H_

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "iCAP_global.h"
#include "imu.h"

/* Check if UART module is enabled */
#ifdef HAL_UART_MODULE_ENABLED

/* Buffer sizes - DMA-based reception for UART4 IMU */
#define RX_DMA_BUFFER_SIZE     4096    /* DMA circular buffer increased to 4KB */

/* UART Status */
typedef enum {
    UART_OK = 0,
    UART_BUSY,
    UART_ERROR,
    UART_TIMEOUT,
    UART_INVALID_CHANNEL
} UART_Status_t;


/* Function Prototypes */

/**
 * @brief Initialize UART channel
 * @param Channel: UART channel ID (UART_UMBILICAL, UART_RPI, etc.)
 * @param baudrate: Baudrate (9600, 115200, etc.)
 * @return true if successful, false otherwise
 */
bool UART_Init(uint16_t Channel, uint32_t baudrate);

/**
 * @brief Transmit data via UART (IT, non-blocking)
 * @param Channel: UART channel ID
 * @param data: Pointer to data buffer
 * @param length: Data length in bytes
 * @return UART_Status_t: UART_OK if transmission started, UART_BUSY if busy, UART_ERROR on error
 */
UART_Status_t UART_Transmit(uint16_t Channel, uint8_t *data, uint16_t length);

/**
 * @brief Check if transmission is complete
 * @param Channel: UART channel ID
 * @return true if transmission complete, false if still transmitting
 */
bool UART_IsTransmitComplete(uint16_t Channel);

/**
 * @brief Get number of bytes available in receive buffer
 * @param Channel: UART channel ID
 * @return Number of bytes available
 */
uint16_t UART_GetAvailableBytes(uint16_t Channel);

/**
 * @brief Receive data from UART buffer (non-blocking)
 * @param Channel: UART channel ID
 * @param buffer: Pointer to receive buffer
 * @param max_length: Maximum length to read
 * @return Number of bytes actually received
 */
uint16_t UART_Receive(uint16_t Channel, uint8_t *buffer, uint16_t max_length);

/**
 * @brief Clear receive buffer for a channel
 * @param Channel: UART channel ID
 */
void UART_ClearBuffer(uint16_t Channel);

/**
 * @brief Get completed packet from interrupt handler (only for UART_IMU channel)
 * @param Channel: UART channel ID (should be UART_IMU)
 * @param packet: Pointer to buffer to store packet (must be at least 44 bytes)
 * @return true if new packet available, false otherwise
 */
bool UART_GetCompletedPacket(uint16_t Channel, uint8_t *packet);

/**
 * @brief Get decoded IMU data (only for UART_IMU channel)
 * @param Channel: UART channel ID (should be UART_IMU)
 * @param imu_data: Pointer to IMU_Data_t structure to fill
 * @return true if new data available, false otherwise
 */
bool UART_GetIMUData(uint16_t Channel, IMU_Data_t *imu_data);

#else
/* UART module not enabled - provide dummy implementations */

/* UART Status (dummy definition when UART is not enabled) */
typedef enum {
    UART_OK = 0,
    UART_BUSY,
    UART_ERROR,
    UART_TIMEOUT,
    UART_INVALID_CHANNEL
} UART_Status_t;

/* Dummy function implementations when UART is not enabled */
static inline bool UART_Init(uint16_t Channel, uint32_t baudrate) { (void)Channel; (void)baudrate; return false; }
static inline UART_Status_t UART_Transmit(uint16_t Channel, uint8_t *data, uint16_t length) { (void)Channel; (void)data; (void)length; return UART_ERROR; }
static inline bool UART_IsTransmitComplete(uint16_t Channel) { (void)Channel; return true; }
static inline uint16_t UART_GetAvailableBytes(uint16_t Channel) { (void)Channel; return 0; }
static inline uint16_t UART_Receive(uint16_t Channel, uint8_t *buffer, uint16_t max_length) { (void)Channel; (void)buffer; (void)max_length; return 0; }
static inline void UART_ClearBuffer(uint16_t Channel) { (void)Channel; }

#endif /* HAL_UART_MODULE_ENABLED */

#endif /* ICAP_INC_UARTCOMM_H_ */

