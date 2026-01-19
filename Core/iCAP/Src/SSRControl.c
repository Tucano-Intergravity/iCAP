/*
 * SSRControl.c
 *
 *  Created on: Jan 6, 2026
 *      Author: USER
 */

#include "SSRControl.h"

bool SSRControl(uint16_t Device, bool state)
{
	switch(Device) {
		case SSR1:
			if (state == true) {
				HAL_GPIO_WritePin(SSR_CONTROL_PORT, SSR1_PIN, GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(SSR_CONTROL_PORT, SSR1_PIN, GPIO_PIN_RESET);
			}
			return true;
		case SSR2:
			if (state == true) {
				HAL_GPIO_WritePin(SSR_CONTROL_PORT, SSR2_PIN, GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(SSR_CONTROL_PORT, SSR2_PIN, GPIO_PIN_RESET);
			}
			return true;
		case SSR3:
			if (state == true) {
				HAL_GPIO_WritePin(SSR_CONTROL_PORT, SSR3_PIN, GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(SSR_CONTROL_PORT, SSR3_PIN, GPIO_PIN_RESET);
			}
			return true;
		case SSR4:
			if (state == true) {
				HAL_GPIO_WritePin(SSR_CONTROL_PORT, SSR4_PIN, GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(SSR_CONTROL_PORT, SSR4_PIN, GPIO_PIN_RESET);
			}
			return true;
		default:
			/* Invalid device ID */
			return false;
	}
}


