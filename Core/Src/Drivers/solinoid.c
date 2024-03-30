/*
 * solinoid.c
 *
 *  Created on: Mar 29, 2024
 *      Author: kiene
 */
#include<solinoid.h>

void enable_solinoid(uint8_t solinoid_id, uint8_t is_enabled) {
	switch (solinoid_id) {
	case 0:
		HAL_GPIO_WritePin(SOL_0_GPIO_Port, SOL_0_Pin, is_enabled);
		break;
	case 1:
		HAL_GPIO_WritePin(SOL_1_GPIO_Port, SOL_1_Pin, is_enabled);
		break;
	case 2:
		HAL_GPIO_WritePin(SOL_2_GPIO_Port, SOL_2_Pin, is_enabled);
		break;
	}
}
