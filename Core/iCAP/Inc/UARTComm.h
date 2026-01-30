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
 * @param Channel: UART channel ID
 * @param baudrate: Baudrate
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
 * @brief Get decoded IMU data (only for UART_IMU channel)
 * @param Channel: UART channel ID (should be UART_IMU)
 * @param imu_data: Pointer to IMU_Data_t structure to fill
 * @return true if new data available, false otherwise
 */
bool UART_GetIMUData(uint16_t Channel, IMU_Data_t *imu_data);

/**
 * @brief Receive data from UART buffer
 * @param Channel: UART channel ID
 * @param buffer: Pointer to destination buffer
 * @param max_length: Maximum number of bytes to read
 * @return uint16_t: Number of bytes actually read
 */
uint16_t UART_Receive(uint16_t Channel, uint8_t *buffer, uint16_t max_length);

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

