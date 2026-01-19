/*
 * PowerControl.h
 *
 *  Created on: Jan 6, 2026
 *      Author: USER
 */

#ifndef ICAP_INC_POWERCONTROL_H_
#define ICAP_INC_POWERCONTROL_H_

#include <stdbool.h>
#include "main.h"
#include "iCAP_global.h"

#define POWER_CONTROL_PORT	GPIOC
#define GNSS_POWER 			GPIO_PIN_0
#define IRIDIUM_POWER 		GPIO_PIN_1
#define IMU_CM4_POWER 		GPIO_PIN_2
#define CERTUS_POWER 		GPIO_PIN_3

#define POWER_ON			true
#define POWER_OFF			false

bool PowerControl(uint16_t Device, bool state);

#endif /* ICAP_INC_POWERCONTROL_H_ */
