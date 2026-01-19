/*
 * UARTComm.c
 *
 *  Created on: Jan 6, 2026
 *      Author: USER
 */

#include "UARTComm.h"

#ifdef HAL_UART_MODULE_ENABLED

#include <string.h>
#include <stddef.h>

/* Extern UART and DMA handles from main.c */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;

/* Buffer sizes - DMA-based reception for UART4 IMU */
#define RX_DMA_BUFFER_SIZE     4096    /* DMA circular buffer increased to 4KB */
#define TX_BUFFER_SIZE         256

/* UART Channel Index Mapping (internal use) */
#define UART_CHANNEL_MAX 7

typedef enum {
    UART_CH_UMBILICAL = 0, /* UART1 */
    UART_CH_DEBUG,         /* UART2 */
    UART_CH_RPI,           /* UART3 */
    UART_CH_IMU,           /* UART4 */
    UART_CH_CERTUS,        /* UART5 */
    UART_CH_GPS,           /* UART7 */
    UART_CH_LTE            /* UART8 */
} UART_Channel_Index_t;

/* UART Instance Structure */
typedef struct {
    UART_HandleTypeDef *huart;           /* UART handle from CubeMX */
    DMA_HandleTypeDef *hdma_rx;          /* DMA handle for RX */
    DMA_HandleTypeDef *hdma_tx;          /* DMA handle for TX */
    
    /* Receive buffer - DMA circular buffer (UART4 IMU) */
    uint8_t rx_dma_buffer[RX_DMA_BUFFER_SIZE];   /* DMA circular buffer */
    volatile uint16_t rx_dma_last_pos;           /* Last processed position in DMA buffer */
    volatile uint16_t rx_dma_current_pos;        /* Current DMA position (for wrap-around detection) */
    
    /* Transmit management */
    volatile bool tx_busy;
    uint8_t tx_buffer[TX_BUFFER_SIZE];
    volatile uint16_t tx_index;
    volatile uint16_t tx_length;
    
    /* Status */
    volatile bool initialized;
    volatile uint32_t error_count;
    volatile uint32_t packet_count;                /* Valid packets received */
    volatile uint32_t sync_error_count;            /* Sync errors */
    volatile uint32_t checksum_error_count;        /* Checksum errors */
    volatile uint32_t data_loss_count;             /* Buffer overflow detected */
    volatile uint16_t last_calculated_checksum;    /* Last calculated checksum for debugging */
    volatile uint16_t last_received_checksum;      /* Last received checksum for debugging */
    volatile uint8_t last_packet_bytes[44];        /* Last packet bytes for debugging */
    
    /* Complete packet buffer (only for UART_IMU) */
    uint8_t completed_packet[IMU_PACKET_SIZE];     /* Completed and verified packet */
    volatile bool packet_ready;                    /* New complete packet available flag */
    
    /* Decoded IMU data (only for UART_IMU) */
    IMU_Data_t decoded_imu_data;
    volatile bool imu_data_ready;                  /* New IMU data available flag */
} UART_Instance_t;

/* Static variables */
static UART_Instance_t uart_instances[UART_CHANNEL_MAX];

/* Forward declarations */
static uint16_t GetChannelIndex(uint16_t Channel);
static void InitChannelMapping(void);
static uint8_t Verify_i400_Checksum(uint8_t *pPacket, uint16_t length);
static uint8_t Verify_i400_Checksum_Linear(uint8_t *pPacket, uint16_t length);
static bool ParseI400Packet(uint16_t channel_idx, uint8_t *packet_data);
static void ProcessDmaBuffer(uint16_t channel_idx);

/* Helper Functions */
static uint16_t GetChannelIndex(uint16_t Channel)
{
    switch (Channel) {
        case UART_UMBILICAL: return UART_CH_UMBILICAL;
        case UART_RPI: return UART_CH_RPI;
        case UART_IMU: return UART_CH_IMU;
        case UART_CERTUS: return UART_CH_CERTUS;
        case UART_GPS: return UART_CH_GPS;
        default: return UART_CHANNEL_MAX;
    }
}

static void InitChannelMapping(void)
{
    /* UART1 (UMBILICAL) */
    uart_instances[0].huart = &huart1;
    uart_instances[0].hdma_rx = &hdma_usart1_rx;
    uart_instances[0].hdma_tx = &hdma_usart1_tx;
    
    /* UART4 (IMU) */
    uart_instances[3].huart = &huart4;
    uart_instances[3].hdma_rx = &hdma_uart4_rx;
    uart_instances[3].hdma_tx = &hdma_uart4_tx;
}

/* Verify i400 IMU Checksum - 16-bit Unsigned Summation (HGNSI Protocol)
 * Algorithm: 16-bit Unsigned Summation (simple cumulative sum)
 * Range: Byte 0 (0x0E Sync Byte) to Byte 41 (42 bytes total)
 * Storage: Little Endian in bytes 42 (LSB) and 43 (MSB)
 * 
 * @param pPacket: Pointer to packet buffer (must be at least 44 bytes)
 * @param length: Packet length (should be IMU_PACKET_SIZE = 44)
 * @return 1 if checksum valid, 0 if invalid
 */
static uint8_t Verify_i400_Checksum(uint8_t *pPacket, uint16_t length)
{
    if (pPacket == NULL || length < IMU_PACKET_SIZE) {
        return 0;  /* Invalid packet */
    }
    
    /* Initialize 16-bit sum to 0 */
    uint16_t sum = 0;
    
    /* Sum 21 words (42 bytes) using Little Endian word formation */
    for (uint16_t i = 0; i < (IMU_PACKET_SIZE - 2); i += 2) {
        uint16_t word = (uint16_t)pPacket[i] | ((uint16_t)pPacket[i + 1] << 8);
        sum += word;
    }
    
    /* Read received checksum from bytes 42 (LSB) and 43 (MSB) - Little Endian */
    uint16_t received_checksum = (uint16_t)pPacket[IMU_PACKET_SIZE - 2] | 
                                 ((uint16_t)pPacket[IMU_PACKET_SIZE - 1] << 8);
    
    /* Compare calculated sum with received checksum */
    return (sum == received_checksum) ? 1 : 0;
}

/* Legacy function name - for compatibility */
static uint8_t Verify_i400_Checksum_Linear(uint8_t *pPacket, uint16_t length)
{
    return Verify_i400_Checksum(pPacket, length);
}

/* Parse i400 Navigation packet (0xA2) - wrapper for IMU_ParsePacket */
static bool ParseI400Packet(uint16_t channel_idx, uint8_t *packet_data)
{
    if (channel_idx != 3) return false;  /* Only process for UART_IMU (UART4) */
    
    UART_Instance_t *inst = &uart_instances[channel_idx];
    
    /* Verify sync byte and message ID first */
    if (packet_data[0] != IMU_SYNC_BYTE || packet_data[1] != IMU_MSG_ID_NAV) {
        return false;
    }
    
    /* Store packet for debugging before verification */
    memcpy((void*)inst->last_packet_bytes, packet_data, IMU_PACKET_SIZE);
    
    /* Calculate checksum for debugging (16-bit Unsigned Summation) */
    uint16_t calculated_sum = 0;
    for (uint16_t i = 0; i < IMU_PACKET_SIZE - 2; i++) {
        calculated_sum += (uint16_t)packet_data[i];
    }
    /* Read received checksum (Little Endian: LSB at index 42, MSB at index 43) */
    uint16_t received_checksum = (uint16_t)packet_data[IMU_PACKET_SIZE - 2] | 
                                  ((uint16_t)packet_data[IMU_PACKET_SIZE - 1] << 8);
    inst->last_calculated_checksum = calculated_sum;
    inst->last_received_checksum = received_checksum;
    
    /* Verify checksum using i400 IMU protocol (16-bit Unsigned Summation) */
    if (Verify_i400_Checksum(packet_data, IMU_PACKET_SIZE) == 0) {
        inst->checksum_error_count++;
        return false;  /* Checksum mismatch */
    }
    
    /* Parse packet using IMU module */
    imu_raw_packet_t *packet = (imu_raw_packet_t *)packet_data;
    
    /* Decode Angular Rate (rad/s) - LSB: 2^-11 rad/s */
    const float ANGULAR_RATE_LSB = 1.0f / (1 << 11);  /* 2^-11 */
    inst->decoded_imu_data.angular_rate_x = (float)packet->angular_rate_x * ANGULAR_RATE_LSB;
    inst->decoded_imu_data.angular_rate_y = (float)packet->angular_rate_y * ANGULAR_RATE_LSB;
    inst->decoded_imu_data.angular_rate_z = (float)packet->angular_rate_z * ANGULAR_RATE_LSB;
    
    /* Decode Linear Acceleration (m/s²) - LSB: (2^-5) * 0.3048 m/s² */
    const float LINEAR_ACCEL_LSB = (1.0f / (1 << 5)) * 0.3048f;  /* (2^-5) * 0.3048 */
    inst->decoded_imu_data.linear_accel_x = (float)packet->linear_accel_x * LINEAR_ACCEL_LSB;
    inst->decoded_imu_data.linear_accel_y = (float)packet->linear_accel_y * LINEAR_ACCEL_LSB;
    inst->decoded_imu_data.linear_accel_z = (float)packet->linear_accel_z * LINEAR_ACCEL_LSB;
    
    /* Status Word */
    inst->decoded_imu_data.status_word = packet->status_word;
    
    /* Decode Delta Angle (rad) - LSB: 2^-33 rad */
    const float DELTA_ANGLE_LSB = 1.0f / (1ULL << 33);  /* 2^-33 */
    inst->decoded_imu_data.delta_angle_x = (float)packet->delta_angle_x * DELTA_ANGLE_LSB;
    inst->decoded_imu_data.delta_angle_y = (float)packet->delta_angle_y * DELTA_ANGLE_LSB;
    inst->decoded_imu_data.delta_angle_z = (float)packet->delta_angle_z * DELTA_ANGLE_LSB;
    
    /* Decode Delta Velocity (m/s) - LSB: (2^-27) * 0.3048 m/s */
    const float DELTA_VELOCITY_LSB = (1.0f / (1ULL << 27)) * 0.3048f;  /* (2^-27) * 0.3048 */
    inst->decoded_imu_data.delta_velocity_x = (float)packet->delta_velocity_x * DELTA_VELOCITY_LSB;
    inst->decoded_imu_data.delta_velocity_y = (float)packet->delta_velocity_y * DELTA_VELOCITY_LSB;
    inst->decoded_imu_data.delta_velocity_z = (float)packet->delta_velocity_z * DELTA_VELOCITY_LSB;
    
    /* Timestamp is not provided by packet, leave it to caller to fill if needed */
    inst->decoded_imu_data.timestamp = 0;
    
    /* Mark data as ready */
    inst->imu_data_ready = true;
    inst->packet_count++;
    
    return true;
}

/* Process DMA circular buffer and extract packets
 * This function processes new data in DMA circular buffer
 * 
 * @param channel_idx: UART channel index
 */
static void ProcessDmaBuffer(uint16_t channel_idx)
{
    UART_Instance_t *inst = &uart_instances[channel_idx];
    
    if (channel_idx != UART_CH_IMU) return;

    /* Get current DMA position */
    uint16_t current_pos = RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(inst->huart->hdmarx);
    inst->rx_dma_current_pos = current_pos;

    /* Calculate available bytes to process */
    uint16_t available;
    if (current_pos >= inst->rx_dma_last_pos) {
        available = current_pos - inst->rx_dma_last_pos;
    } else {
        available = (RX_DMA_BUFFER_SIZE - inst->rx_dma_last_pos) + current_pos;
    }

    /* Process buffer while we have enough data for a packet header at least */
    while (available >= IMU_PACKET_SIZE) {
        uint16_t pos = inst->rx_dma_last_pos;

        /* Check for Sync Byte (0x0E) */
        if (inst->rx_dma_buffer[pos] == IMU_SYNC_BYTE) {
            /* Found 0x0E, now check for 0xA2 Message ID at next position */
            uint16_t next_pos = (pos + 1) % RX_DMA_BUFFER_SIZE;
            
            if (inst->rx_dma_buffer[next_pos] == IMU_MSG_ID_NAV) {
                /* We have 0x0E and 0xA2. Extract full 44 bytes (handle wrap-around) */
                uint8_t packet[IMU_PACKET_SIZE];
                for (uint16_t i = 0; i < IMU_PACKET_SIZE; i++) {
                    packet[i] = inst->rx_dma_buffer[(pos + i) % RX_DMA_BUFFER_SIZE];
                }

                /* Verify checksum FIRST using 16-bit Word Sum (Method 3) */
                uint16_t sum = 0;
                for (uint16_t j = 0; j < (IMU_PACKET_SIZE - 2); j += 2) {
                    /* Create 16-bit word using Little Endian (pPacket[i+1] << 8 | pPacket[i]) */
                    uint16_t word = (uint16_t)packet[j] | ((uint16_t)packet[j + 1] << 8);
                    sum += word;
                }
                uint16_t received_checksum = (uint16_t)packet[42] | ((uint16_t)packet[43] << 8);

                if (sum == received_checksum) {
                    /* Valid packet! Update storage and pointers */
                    for (uint16_t j = 0; j < IMU_PACKET_SIZE; j++) {
                        inst->completed_packet[j] = packet[j];
                    }
                    inst->packet_ready = true;
                    inst->packet_count++;

                    /* Toggle PA2 to indicate valid IMU data reception */
                    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);

                    /* Advance last_pos by full packet size */
                    inst->rx_dma_last_pos = (pos + IMU_PACKET_SIZE) % RX_DMA_BUFFER_SIZE;
                    available -= IMU_PACKET_SIZE;
                    continue; /* Search for next packet */
                } else {
                    /* Checksum failed - False Sync (0x0E was in data) */
                    inst->checksum_error_count++;
                    /* Advance by only 1 byte to search from next position */
                    inst->rx_dma_last_pos = (pos + 1) % RX_DMA_BUFFER_SIZE;
                    available--;
                    continue;
                }
            } else {
                /* Not 0xA2 after 0x0E - Skip this 0x0E */
                inst->sync_error_count++;
                inst->rx_dma_last_pos = (pos + 1) % RX_DMA_BUFFER_SIZE;
                available--;
                continue;
            }
        }
        
        /* Not 0x0E - Skip to next byte */
        inst->rx_dma_last_pos = (pos + 1) % RX_DMA_BUFFER_SIZE;
        available--;
    }
}

/* Public Functions */

bool UART_Init(uint16_t Channel, uint32_t baudrate)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    if (channel_idx >= UART_CHANNEL_MAX) {
        return false;
    }
    
    InitChannelMapping();
    
    UART_Instance_t *inst = &uart_instances[channel_idx];
    
    /* Configure UART baudrate */
    inst->huart->Init.BaudRate = baudrate;
    if (HAL_UART_Init(inst->huart) != HAL_OK) {
        return false;
    }
    
    /* Initialize instance */
    inst->tx_busy = false;
    inst->tx_index = 0;
    inst->tx_length = 0;
    inst->initialized = false;
    inst->error_count = 0;
    inst->packet_count = 0;
    inst->sync_error_count = 0;
    inst->checksum_error_count = 0;
    inst->data_loss_count = 0;
    inst->imu_data_ready = false;
    inst->packet_ready = false;
    
    /* Initialize DMA buffer pointers */
    inst->rx_dma_last_pos = 0;
    inst->rx_dma_current_pos = 0;
    memset(inst->rx_dma_buffer, 0, RX_DMA_BUFFER_SIZE);
    memset(inst->completed_packet, 0, IMU_PACKET_SIZE);
    
    /* For UART4 (IMU), start DMA reception in Circular mode */
    if (channel_idx == UART_CH_IMU) {
        /* Clear any pending IDLE flag */
        __HAL_UART_CLEAR_IDLEFLAG(inst->huart);
        
        /* Enable IDLE line interrupt for packet detection */
        __HAL_UART_ENABLE_IT(inst->huart, UART_IT_IDLE);
        
        /* Start DMA reception in Circular mode */
        if (HAL_UART_Receive_DMA(inst->huart, inst->rx_dma_buffer, RX_DMA_BUFFER_SIZE) != HAL_OK) {
            return false;
        }
    }
    
    inst->initialized = true;
    return true;
}

UART_Status_t UART_Transmit(uint16_t Channel, uint8_t *data, uint16_t length)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    if (channel_idx >= UART_CHANNEL_MAX || data == NULL || length == 0) {
        return UART_INVALID_CHANNEL;
    }
    
    UART_Instance_t *inst = &uart_instances[channel_idx];
    if (!inst->initialized) {
        return UART_ERROR;
    }
    
    if (inst->tx_busy) {
        return UART_BUSY;
    }
    
    if (length > TX_BUFFER_SIZE) {
        length = TX_BUFFER_SIZE;
    }
    
    /* Copy data to transmit buffer */
    memcpy(inst->tx_buffer, data, length);
    inst->tx_index = 0;
    inst->tx_length = length;
    inst->tx_busy = true;
    
    /* Start transmission */
    if (HAL_UART_Transmit_IT(inst->huart, inst->tx_buffer, length) != HAL_OK) {
        inst->tx_busy = false;
        return UART_ERROR;
    }
    
    return UART_OK;
}

bool UART_IsTransmitComplete(uint16_t Channel)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    if (channel_idx >= UART_CHANNEL_MAX) {
        return true;
    }
    
    UART_Instance_t *inst = &uart_instances[channel_idx];
    return !inst->tx_busy;
}

uint16_t UART_GetAvailableBytes(uint16_t Channel)
{
    /* For double buffering mode (UART4 IMU), this function is not used
     * as data is processed directly in HAL callbacks */
    (void)Channel;
    return 0;
}

uint16_t UART_Receive(uint16_t Channel, uint8_t *buffer, uint16_t max_length)
{
    /* For double buffering mode (UART4 IMU), this function is not used
     * as data is processed directly in HAL callbacks */
    (void)Channel;
    (void)buffer;
    (void)max_length;
    return 0;
}

void UART_ClearBuffer(uint16_t Channel)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    if (channel_idx >= UART_CHANNEL_MAX) {
        return;
    }
    
    UART_Instance_t *inst = &uart_instances[channel_idx];
    inst->imu_data_ready = false;
    inst->packet_count = 0;
    inst->sync_error_count = 0;
    inst->checksum_error_count = 0;
    inst->data_loss_count = 0;
}

/* Get completed packet from interrupt handler (non-blocking)
 * @param Channel: UART channel ID (should be UART_IMU)
 * @param packet: Pointer to buffer to store packet (must be at least 44 bytes)
 * @return true if new packet available, false otherwise
 */
bool UART_GetCompletedPacket(uint16_t Channel, uint8_t *packet)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    
    /* Only available for UART_IMU (UART4) */
    if (channel_idx != UART_CH_IMU || packet == NULL) {
        return false;
    }
    
    UART_Instance_t *inst = &uart_instances[channel_idx];
    
    if (!inst->initialized) {
        return false;
    }
    
    /* Check if new complete packet is available */
    if (!inst->packet_ready) {
        return false;
    }
    
    /* Copy completed packet */
    for (uint16_t i = 0; i < IMU_PACKET_SIZE; i++) {
        packet[i] = inst->completed_packet[i];
    }
    
    /* Clear ready flag */
    inst->packet_ready = false;
    
    return true;
}

bool UART_GetIMUData(uint16_t Channel, IMU_Data_t *imu_data)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    
    /* Only available for UART_IMU (UART4) */
    if (channel_idx != UART_CH_IMU || imu_data == NULL) {
        return false;
    }
    
    UART_Instance_t *inst = &uart_instances[channel_idx];
    
    if (!inst->initialized) {
        return false;
    }
    
    /* Data is processed in HAL_UARTEx_RxEventCallback (IDLE interrupt)
     * Check if new complete packet was prepared by ISR */
    if (!inst->packet_ready) {
        return false;
    }
    
    /* Parse completed packet into physical values */
    if (ParseI400Packet(channel_idx, inst->completed_packet)) {
        /* Copy decoded data */
        *imu_data = inst->decoded_imu_data;
        
        /* Clear packet ready flag after consuming */
        inst->packet_ready = false;
        return true;
    }
    
    /* Parsing failed - clear flag anyway */
    inst->packet_ready = false;
    return false;
}

/* HAL UART Callbacks */

/* IDLE Line Callback - Called when IDLE line detected (DMA mode) */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* Find which channel this UART belongs to */
    for (uint16_t i = 0; i < UART_CHANNEL_MAX; i++) {
        if (uart_instances[i].huart == huart && uart_instances[i].initialized) {
            /* For UART4 (IMU), process DMA buffer when IDLE detected */
            if (i == UART_CH_IMU) {
                /* Process new data in DMA buffer */
                ProcessDmaBuffer(i);
            }
            break;
        }
    }
}

/* Transmit Complete Callback */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Find which channel this UART belongs to */
    for (uint16_t i = 0; i < UART_CHANNEL_MAX; i++) {
        if (uart_instances[i].huart == huart) {
            uart_instances[i].tx_busy = false;
            uart_instances[i].tx_index = 0;
            uart_instances[i].tx_length = 0;
            break;
        }
    }
}

/* Error Callback */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Find which channel this UART belongs to */
    for (uint16_t i = 0; i < UART_CHANNEL_MAX; i++) {
        if (uart_instances[i].huart == huart && uart_instances[i].initialized) {
            /* Check for overrun error */
            if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
                /* Clear overrun flag immediately using direct register access for reliability */
                __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
                
                /* For UART_IMU (UART4), restart DMA and reset processing pointers */
                if (i == UART_CH_IMU) {
                    HAL_UART_DMAStop(huart);
                    
                    /* Reset pointers to avoid processing corrupted or legacy data */
                    uart_instances[i].rx_dma_last_pos = 0;
                    uart_instances[i].rx_dma_current_pos = 0;
                    
                    HAL_UART_Receive_DMA(huart, uart_instances[i].rx_dma_buffer, RX_DMA_BUFFER_SIZE);
                }
            }
            
            uart_instances[i].error_count++;
            break;
        }
    }
}

#endif /* HAL_UART_MODULE_ENABLED */
