/*
 * PowerControl.c
 *
 *  Created on: Jan 6, 2026
 *      Author: USER
 */

#include "PowerControl.h"

bool PowerControl(uint16_t Device, bool state)
{
	switch(Device) {
		case GNSS:
			if (state == true) {
				HAL_GPIO_WritePin(POWER_CONTROL_PORT, GNSS_POWER, GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(POWER_CONTROL_PORT, GNSS_POWER, GPIO_PIN_RESET);
			}
			return true;
		case IRIDIUM:
			if (state == true) {
				HAL_GPIO_WritePin(POWER_CONTROL_PORT, IRIDIUM_POWER, GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(POWER_CONTROL_PORT, IRIDIUM_POWER, GPIO_PIN_RESET);
			}
			return true;
		case IMU:
		case CM4:
			/* IMU and CM4 share the same power control pin */
			if (state == true) {
				HAL_GPIO_WritePin(POWER_CONTROL_PORT, IMU_CM4_POWER, GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(POWER_CONTROL_PORT, IMU_CM4_POWER, GPIO_PIN_RESET);
			}
			return true;
		case CERTUS:
			if (state == true) {
				HAL_GPIO_WritePin(POWER_CONTROL_PORT, CERTUS_POWER, GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(POWER_CONTROL_PORT, CERTUS_POWER, GPIO_PIN_RESET);
			}
			return true;
		default:
			/* Invalid device ID */
			return false;
	}
}


