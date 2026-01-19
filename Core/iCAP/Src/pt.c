/*
 * pt.c
 *
 *  Created on: Jan 19, 2026
 *      Author: AI Assistant
 */

#include "pt.h"
#include "main.h"
#include <stddef.h>

/* main.c에서 생성된 ADC 핸들 참조 */
extern ADC_HandleTypeDef hadc1;

/* [Live Expression용 전역 변수] */
uint16_t adc_raw_values[2] = {0, 0}; 

/* 내부 관리용 센서 데이터 */
static PT_Data_t pt_sensors[4]; 

/**
 * @brief Initialize Pressure Transducer module
 */
void PT_Init(void)
{
    /* ADC 자가 보정 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    
    /* DMA 수신 시작: 2개 채널(PA6, PA7) 데이터를 adc_raw_values에 무한 업데이트 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_values, 2);

    for (int i = 0; i < 4; i++) {
        pt_sensors[i].pressure_bar = 0.0f;
        pt_sensors[i].is_valid = (i < 2) ? true : false; /* PA6, PA7 두 개만 우선 사용 */
    }
}

/**
 * @brief Update PT sensor data (변환 로직)
 */
void PT_Update(void)
{
    /* 16-bit ADC (0-65535) -> 전압 (0-3.3V) -> 압력 변환 */
    /* 예시: 0.5V~4.5V 센서이나 보드 전압 분배에 따라 계수 조정 필요 */
    for (int i = 0; i < 2; i++) {
        pt_sensors[i].raw_adc_value = adc_raw_values[i];
        
        // 간단한 전압 확인용 식
        float voltage = (adc_raw_values[i] / 65535.0f) * 3.3f;
        pt_sensors[i].pressure_bar = voltage * 10.0f; // TODO: 센서 공식 적용
    }
}

/**
 * @brief Get current pressure value
 */
float PT_GetPressure(uint8_t sensor_id)
{
    if (sensor_id >= 4) return 0.0f;
    return pt_sensors[sensor_id].pressure_bar;
}

/**
 * @brief Check if sensor data is valid
 */
bool PT_IsHealthy(uint8_t sensor_id)
{
    if (sensor_id >= 4) return false;
    return pt_sensors[sensor_id].is_valid;
}
