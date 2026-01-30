#include "stm32h7xx_hal.h"
#include "tc.h"
#include <string.h>

/* External SPI Handle from main.c */
extern SPI_HandleTypeDef hspi1;

/* Chip Select Pin Mapping */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} CS_Pin_t;

static const CS_Pin_t TC_CS_PINS[3] = {
    {GPIOG, GPIO_PIN_10}, /* TC1 */
    {GPIOG, GPIO_PIN_11}, /* TC2 */
    {GPIOG, GPIO_PIN_12}  /* TC3 */
};

TC_Data_t tc_sensors[3];

/**
 * @brief Initialize TC Module
 */
void TC_Init(void) {
    memset(tc_sensors, 0, sizeof(tc_sensors));
    
    /* Ensure CS pins are HIGH (Deselected) */
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(TC_CS_PINS[i].port, TC_CS_PINS[i].pin, GPIO_PIN_SET);
    }
}

/**
 * @brief Parse MAX31855 Raw Data
 */
static void TC_ParseMAX31855(uint8_t index, uint8_t* raw) {
    uint32_t data = ((uint32_t)raw[0] << 24) | 
                    ((uint32_t)raw[1] << 16) | 
                    ((uint32_t)raw[2] << 8)  | 
                    (uint32_t)raw[3];

    TC_Data_t* sensor = &tc_sensors[index];

    /* Check for Faults (Bit 16) */
    if (data & 0x00010000) {
        sensor->faults.fault_any = true;
        sensor->faults.short_vcc = (data & 0x00000004) ? true : false;
        sensor->faults.short_gnd = (data & 0x00000002) ? true : false;
        sensor->faults.open_circuit = (data & 0x00000001) ? true : false;
        sensor->is_valid = false;
    } else {
        /* No fault: Parse TC Temperature (Bits 31:18, signed 14-bit) */
        int16_t tc_raw = (int16_t)((data >> 18) & 0x3FFF);
        if (tc_raw & 0x2000) { /* Handle negative values */
            tc_raw |= 0xC000;
        }
        sensor->temperature = tc_raw * 0.25f;

        /* Parse Internal Temperature (Bits 15:4, signed 12-bit) */
        int16_t int_raw = (int16_t)((data >> 4) & 0x0FFF);
        if (int_raw & 0x0800) { /* Handle negative values */
            int_raw |= 0xF000;
        }
        sensor->internal_temp = int_raw * 0.0625f;

        sensor->faults.fault_any = false;
        sensor->is_valid = true;
        sensor->last_update_tick = HAL_GetTick();
    }
}

/**
 * @brief Read all 3 TC sensors via SPI1
 */
void TC_Update(void) {
    uint8_t rx_buf[4];
    HAL_StatusTypeDef status;
    
    for (int i = 0; i < 3; i++) {
        /* 1. Clear any pending SPI errors to prevent hang */
        if (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_OVR) != RESET) {
            __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
        }

        /* 2. Select Sensor */
        HAL_GPIO_WritePin(TC_CS_PINS[i].port, TC_CS_PINS[i].pin, GPIO_PIN_RESET);
        
        /* Small delay for MAX31855 CS setup time */
        for(volatile int d=0; d<100; d++);

        /* 3. Read 32 bits (4 bytes) */
        status = HAL_SPI_Receive(&hspi1, rx_buf, 4, 10);
        
        if (status == HAL_OK) {
            TC_ParseMAX31855(i, rx_buf);
        } else {
            /* If SPI fails, try to abort to clear Busy state */
            HAL_SPI_Abort(&hspi1);
            tc_sensors[i].is_valid = false;
        }
        
        /* 4. Deselect Sensor */
        HAL_GPIO_WritePin(TC_CS_PINS[i].port, TC_CS_PINS[i].pin, GPIO_PIN_SET);
        
        /* Small delay before next sensor access */
        for(volatile int d=0; d<100; d++);
    }
}

float TC_GetTemperature(uint8_t index) {
    if (index < 3) return tc_sensors[index].temperature;
    return -999.0f;
}

bool TC_IsHealthy(uint8_t index) {
    if (index < 3) return tc_sensors[index].is_valid;
    return false;
}
