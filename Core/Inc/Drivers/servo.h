/*
 * servo.h
 *
 *  Created on: Mar 29, 2024
 *      Author: kiene
 */

#ifndef SERVO_H_
#define SERVO_H_

#define SERV_1_Pin GPIO_PIN_0
#define SERV_1_GPIO_Port GPIOA
#define SERV_2_Pin GPIO_PIN_1
#define SERV_2_GPIO_Port GPIOA
#define SERV_3_Pin GPIO_PIN_2
#define SERV_3_GPIO_Port GPIOA
#define PWM_1_Pin GPIO_PIN_7
#define PWM_1_GPIO_Port GPIOB
#define PWM_2_Pin GPIO_PIN_8
#define PWM_2_GPIO_Port GPIOB
#define PWM_3_Pin GPIO_PIN_9
#define PWM_3_GPIO_Port GPIOB
#define SERV_CTL_1_Pin GPIO_PIN_7
#define SERV_CTL_1_GPIO_Port GPIOD
#define SERV_CTL_2_Pin GPIO_PIN_8
#define SERV_CTL_2_GPIO_Port GPIOD
#define SERV_CTL_3_Pin GPIO_PIN_9
#define SERV_CTL_3_GPIO_Port GPIOD

#define MIN_ANGLE 0   // Min Servo Angle
#define MAX_ANGLE 180 // Max Servo Angle

#define SERVOMIN  544 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2400 // This is the 'maximum' pulse length count (out of 4096)

#define ANGLE_TO_US(a) ((uint16_t)((((a) - MIN_ANGLE) * (SERVOMAX - SERVOMIN) / (MAX_ANGLE - MIN_ANGLE)) + SERVOMIN))


#endif /* INC_DRIVERS_SERVO_H_ */
