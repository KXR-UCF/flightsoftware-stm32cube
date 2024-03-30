/*
 * servo.c
 *
 *  Created on: Mar 29, 2024
 *      Author: kiene
 */
#include <servo.h>
#include <tim.h>

void enable_servo(uint8_t servo_id, uint8_t is_enabled) {
	switch (servo_id) {
	case 1:
		HAL_GPIO_WritePin((GPIO_TypeDef*) SERV_CTL_1_GPIO_Port, SERV_CTL_1_Pin,
				is_enabled);
		break;
	case 2:
		HAL_GPIO_WritePin((GPIO_TypeDef*) SERV_CTL_2_GPIO_Port, SERV_CTL_2_Pin,
				is_enabled);
		break;
	case 3:
		HAL_GPIO_WritePin((GPIO_TypeDef*) SERV_CTL_3_GPIO_Port, SERV_CTL_3_Pin,
				is_enabled);
		break;
	}

	void set_servo_angle(int8_t servo_id, uint16_t angle) {
		switch (servo_id) {
		case 1:
			TIM4->CCR2 = ANGLE_TO_US(angle);
			break;
		case 2:
			TIM4->CCR3 = ANGLE_TO_US(angle);
			break;
		case 3:
			TIM4->CCR4 = ANGLE_TO_US(angle);
			break;
		}
	}
