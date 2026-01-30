#ifndef __TC_H
#define __TC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief MAX31855 Fault bits
 */
typedef struct {
    bool fault_any;     /* Any fault occurred */
    bool short_vcc;     /* Short to VCC */
    bool short_gnd;     /* Short to GND */
    bool open_circuit;  /* Open circuit */
} TC_Fault_t;

/**
 * @brief Thermocouple Data structure for MAX31855
 */
typedef struct {
    float temperature;          /* TC Temperature in Celsius */
    float internal_temp;        /* Internal Reference Junction Temp */
    TC_Fault_t faults;          /* Diagnostic info */
    bool is_valid;              /* Data validity flag */
    uint32_t last_update_tick;  /* Last successful read time */
} TC_Data_t;

/* Public Function Prototypes */
void TC_Init(void);
void TC_Update(void);
float TC_GetTemperature(uint8_t index);
bool TC_IsHealthy(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* __TC_H */
