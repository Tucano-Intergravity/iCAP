/*
 * SolControl.h
 *
 *  Created on: Jan 6, 2026
 *      Author: USER
 */

#ifndef ICAP_INC_SSRCONTROL_H_
#define ICAP_INC_SSRCONTROL_H_

#include <stdbool.h>
#include "main.h"
#include "iCAP_global.h"

#define SSR_CONTROL_PORT	GPIOD
#define SSR1_PIN			GPIO_PIN_0
#define SSR2_PIN			GPIO_PIN_1
#define SSR3_PIN 			GPIO_PIN_2
#define SSR4_PIN 			GPIO_PIN_3

#define SSR_ON				true
#define SSR_OFF				false

bool SSRControl(uint16_t Device, bool state);

#endif /* ICAP_INC_SSRCONTROL_H_ */
