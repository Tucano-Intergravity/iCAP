/*
 * pt.h
 *
 *  Created on: Jan 19, 2026
 *      Author: AI Assistant
 */

#ifndef ICAP_INC_PT_H_
#define ICAP_INC_PT_H_

#include <stdint.h>
#include <stdbool.h>

/* Pressure Transducer Data Structure */
typedef struct {
    float pressure_bar;      /* Pressure in Bar */
    float temperature_c;     /* Temperature in Celsius (if supported) */
    uint32_t raw_adc_value;  /* Raw ADC value */
    uint32_t timestamp;      /* Last update timestamp */
    bool is_valid;           /* Data validity flag */
} PT_Data_t;

/* Function Prototypes */
void PT_Init(void);
void PT_Update(void);
float PT_GetPressure(uint8_t sensor_id);
bool PT_IsHealthy(uint8_t sensor_id);

/* Live Expression용 외부 참조 선언 */
extern uint16_t adc_raw_values[2];

#endif /* ICAP_INC_PT_H_ */
