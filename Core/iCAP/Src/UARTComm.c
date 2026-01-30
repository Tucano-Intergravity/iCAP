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
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;

/* Buffer sizes - DMA-based reception for UART4 IMU */
#define RX_DMA_BUFFER_SIZE     4096    /* DMA circular buffer increased to 4KB */
#define TX_BUFFER_SIZE         256

/* UART Channel Index Mapping (internal use) */
#define UART_CHANNEL_MAX 7

typedef enum {
    UART_CH_UMBILICAL = 0, /* UART1 */
    UART_CH_DEBUG,         /* UART2 */
    UART_CH_RPI,           /* UART3 (Mapped to UART5) */
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
    
    /* Software Ring Buffer for non-DMA channels (UART5, etc.) */
    uint8_t rx_sw_buffer[512];
    volatile uint16_t rx_sw_head;
    volatile uint16_t rx_sw_tail;
    uint8_t rx_it_byte;                  /* Byte for IT reception */

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
static bool ParseI400Packet(uint16_t channel_idx, uint8_t *packet_data);
static void ProcessDmaBuffer(uint16_t channel_idx);

/* Helper Functions */
static uint16_t GetChannelIndex(uint16_t Channel)
{
    switch (Channel) {
        case UART_UMBILICAL: return UART_CH_UMBILICAL;
        case UART_RPI: return UART_CH_CERTUS;  /* Map RPI to Index 4 (UART5) */
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

    /* UART5 (RPI/CERTUS) */
    uart_instances[4].huart = &huart5;
    uart_instances[4].hdma_rx = &hdma_uart5_rx;
    uart_instances[4].hdma_tx = &hdma_uart5_tx;
}

/* Verify i400 IMU Checksum - 16-bit Word Sum (Method 3) */
static uint8_t Verify_i400_Checksum(uint8_t *pPacket, uint16_t length)
{
    if (pPacket == NULL || length < IMU_PACKET_SIZE) {
        return 0;
    }
    uint16_t sum = 0;
    for (uint16_t i = 0; i < (IMU_PACKET_SIZE - 2); i += 2) {
        uint16_t word = (uint16_t)pPacket[i] | ((uint16_t)pPacket[i + 1] << 8);
        sum += word;
    }
    uint16_t received_checksum = (uint16_t)pPacket[42] | ((uint16_t)pPacket[43] << 8);
    return (sum == received_checksum) ? 1 : 0;
}

/* Parse i400 Navigation packet (0xA2) */
static bool ParseI400Packet(uint16_t channel_idx, uint8_t *packet_data)
{
    if (channel_idx != UART_CH_IMU) return false;
    UART_Instance_t *inst = &uart_instances[channel_idx];
    
    if (packet_data[0] != IMU_SYNC_BYTE || packet_data[1] != IMU_MSG_ID_NAV) return false;
    
    memcpy((void*)inst->last_packet_bytes, packet_data, IMU_PACKET_SIZE);
    
    // [REVERTED] Use structure casting as requested
    imu_raw_packet_t *packet = (imu_raw_packet_t *)packet_data;
    
    const float ANGULAR_RATE_LSB = 1.0f / (1 << 11);
    inst->decoded_imu_data.angular_rate_x = (float)packet->angular_rate_x * ANGULAR_RATE_LSB;
    inst->decoded_imu_data.angular_rate_y = (float)packet->angular_rate_y * ANGULAR_RATE_LSB;
    inst->decoded_imu_data.angular_rate_z = (float)packet->angular_rate_z * ANGULAR_RATE_LSB;
    
    const float LINEAR_ACCEL_LSB = (1.0f / (1 << 5)) * 0.3048f;
    inst->decoded_imu_data.linear_accel_x = (float)packet->linear_accel_x * LINEAR_ACCEL_LSB;
    inst->decoded_imu_data.linear_accel_y = (float)packet->linear_accel_y * LINEAR_ACCEL_LSB;
    inst->decoded_imu_data.linear_accel_z = (float)packet->linear_accel_z * LINEAR_ACCEL_LSB;
    
    inst->decoded_imu_data.status_word = packet->status_word;
    
    const float DELTA_ANGLE_LSB = 1.0f / (1ULL << 33);
    inst->decoded_imu_data.delta_angle_x = (float)packet->delta_angle_x * DELTA_ANGLE_LSB;
    inst->decoded_imu_data.delta_angle_y = (float)packet->delta_angle_y * DELTA_ANGLE_LSB;
    inst->decoded_imu_data.delta_angle_z = (float)packet->delta_angle_z * DELTA_ANGLE_LSB;
    
    const float DELTA_VELOCITY_LSB = (1.0f / (1ULL << 27)) * 0.3048f;
    inst->decoded_imu_data.delta_velocity_x = (float)packet->delta_velocity_x * DELTA_VELOCITY_LSB;
    inst->decoded_imu_data.delta_velocity_y = (float)packet->delta_velocity_y * DELTA_VELOCITY_LSB;
    inst->decoded_imu_data.delta_velocity_z = (float)packet->delta_velocity_z * DELTA_VELOCITY_LSB;
    
    inst->decoded_imu_data.timestamp = 0;
    inst->imu_data_ready = true;
    inst->packet_count++;
    return true;
}

static void ProcessDmaBuffer(uint16_t channel_idx)
{
    UART_Instance_t *inst = &uart_instances[channel_idx];
    if (channel_idx != UART_CH_IMU) return;

    uint16_t current_pos = RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(inst->huart->hdmarx);
    inst->rx_dma_current_pos = current_pos;

    uint16_t available;
    if (current_pos >= inst->rx_dma_last_pos) {
        available = current_pos - inst->rx_dma_last_pos;
    } else {
        available = (RX_DMA_BUFFER_SIZE - inst->rx_dma_last_pos) + current_pos;
    }

    while (available >= IMU_PACKET_SIZE) {
        uint16_t pos = inst->rx_dma_last_pos;
        if (inst->rx_dma_buffer[pos] == IMU_SYNC_BYTE) {
            uint16_t next_pos = (pos + 1) % RX_DMA_BUFFER_SIZE;
            if (inst->rx_dma_buffer[next_pos] == IMU_MSG_ID_NAV) {
                uint8_t packet[IMU_PACKET_SIZE];
                for (uint16_t i = 0; i < IMU_PACKET_SIZE; i++) {
                    packet[i] = inst->rx_dma_buffer[(pos + i) % RX_DMA_BUFFER_SIZE];
                }

                uint16_t sum = 0;
                for (uint16_t j = 0; j < (IMU_PACKET_SIZE - 2); j += 2) {
                    uint16_t word = (uint16_t)packet[j] | ((uint16_t)packet[j + 1] << 8);
                    sum += word;
                }
                uint16_t received_checksum = (uint16_t)packet[42] | ((uint16_t)packet[43] << 8);

                if (sum == received_checksum) {
                    memcpy(inst->completed_packet, packet, IMU_PACKET_SIZE);
                    inst->packet_ready = true;
                    inst->packet_count++;
                    inst->rx_dma_last_pos = (pos + IMU_PACKET_SIZE) % RX_DMA_BUFFER_SIZE;
                    available -= IMU_PACKET_SIZE;
                    continue;
                }
            }
        }
        inst->rx_dma_last_pos = (pos + 1) % RX_DMA_BUFFER_SIZE;
        available--;
    }
}

bool UART_Init(uint16_t Channel, uint32_t baudrate)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    if (channel_idx >= UART_CHANNEL_MAX) return false;
    
    InitChannelMapping();
    UART_Instance_t *inst = &uart_instances[channel_idx];
    inst->huart->Init.BaudRate = baudrate;
    if (HAL_UART_Init(inst->huart) != HAL_OK) return false;
    
    inst->initialized = false;
    inst->packet_count = 0;
    inst->rx_sw_head = 0;
    inst->rx_sw_tail = 0;
    
    if (channel_idx == UART_CH_IMU) {
        __HAL_UART_CLEAR_IDLEFLAG(inst->huart);
        __HAL_UART_ENABLE_IT(inst->huart, UART_IT_IDLE);
        if (HAL_UART_Receive_DMA(inst->huart, inst->rx_dma_buffer, RX_DMA_BUFFER_SIZE) != HAL_OK) return false;
    } else {
        /* Start Interrupt reception for RPI/UART5 */
        HAL_UART_Receive_IT(inst->huart, &inst->rx_it_byte, 1);
    }
    
    inst->initialized = true;
    return true;
}

UART_Status_t UART_Transmit(uint16_t Channel, uint8_t *data, uint16_t length)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    if (channel_idx >= UART_CHANNEL_MAX || data == NULL || length == 0) return UART_INVALID_CHANNEL;
    UART_Instance_t *inst = &uart_instances[channel_idx];
    if (!inst->initialized || inst->tx_busy) return UART_ERROR;
    
    if (length > TX_BUFFER_SIZE) length = TX_BUFFER_SIZE;
    memcpy(inst->tx_buffer, data, length);
    inst->tx_busy = true;
    if (HAL_UART_Transmit_IT(inst->huart, inst->tx_buffer, length) != HAL_OK) {
        inst->tx_busy = false;
        return UART_ERROR;
    }
    return UART_OK;
}

bool UART_IsTransmitComplete(uint16_t Channel)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    if (channel_idx >= UART_CHANNEL_MAX) return true;
    return !uart_instances[channel_idx].tx_busy;
}

uint16_t UART_Receive(uint16_t Channel, uint8_t *buffer, uint16_t max_length)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    if (channel_idx >= UART_CHANNEL_MAX || buffer == NULL) return 0;
    UART_Instance_t *inst = &uart_instances[channel_idx];
    
    uint16_t count = 0;
    while (count < max_length && inst->rx_sw_tail != inst->rx_sw_head) {
        buffer[count++] = inst->rx_sw_buffer[inst->rx_sw_tail];
        inst->rx_sw_tail = (inst->rx_sw_tail + 1) % 512;
    }
    return count;
}

bool UART_GetIMUData(uint16_t Channel, IMU_Data_t *imu_data)
{
    uint16_t channel_idx = GetChannelIndex(Channel);
    if (channel_idx != UART_CH_IMU || imu_data == NULL) return false;
    UART_Instance_t *inst = &uart_instances[channel_idx];
    if (!inst->initialized || !inst->packet_ready) return false;
    
    if (ParseI400Packet(channel_idx, inst->completed_packet)) {
        *imu_data = inst->decoded_imu_data;
        inst->packet_ready = false;
        return true;
    }
    inst->packet_ready = false;
    return false;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint16_t i = 0; i < UART_CHANNEL_MAX; i++) {
        if (uart_instances[i].huart == huart && i == UART_CH_IMU) {
            ProcessDmaBuffer(i);
            break;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint16_t i = 0; i < UART_CHANNEL_MAX; i++) {
        if (uart_instances[i].huart == huart && i != UART_CH_IMU) {
            UART_Instance_t *inst = &uart_instances[i];
            uint16_t next_head = (inst->rx_sw_head + 1) % 512;
            if (next_head != inst->rx_sw_tail) {
                inst->rx_sw_buffer[inst->rx_sw_head] = inst->rx_it_byte;
                inst->rx_sw_head = next_head;
            }
            HAL_UART_Receive_IT(huart, &inst->rx_it_byte, 1);
            break;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for (uint16_t i = 0; i < UART_CHANNEL_MAX; i++) {
        if (uart_instances[i].huart == huart) {
            uart_instances[i].tx_busy = false;
            break;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint16_t i = 0; i < UART_CHANNEL_MAX; i++) {
        if (uart_instances[i].huart == huart && uart_instances[i].initialized) {
            if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
                __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
                if (i == UART_CH_IMU) {
                    HAL_UART_DMAStop(huart);
                    uart_instances[i].rx_dma_last_pos = 0;
                    HAL_UART_Receive_DMA(huart, uart_instances[i].rx_dma_buffer, RX_DMA_BUFFER_SIZE);
                } else {
                    HAL_UART_Receive_IT(huart, &uart_instances[i].rx_it_byte, 1);
                }
            }
            uart_instances[i].error_count++;
            break;
        }
    }
}

#endif /* HAL_UART_MODULE_ENABLED */
