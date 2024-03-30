/*
 * pressure_transducer.h
 *
 *  Created on: Mar 29, 2024
 *      Author: kiene
 */

#ifndef PRESSURE_TRANSDUCER_H_
#define PRESSURE_TRANSDUCER_H_

#define PT_CH_7_Pin GPIO_PIN_4
#define PT_CH_7_GPIO_Port GPIOA
#define PT_CH_6_Pin GPIO_PIN_5
#define PT_CH_6_GPIO_Port GPIOA
#define PT_CH_5_Pin GPIO_PIN_6
#define PT_CH_5_GPIO_Port GPIOA
#define PT_CH_4_Pin GPIO_PIN_7
#define PT_CH_4_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_8
#define PT_CH_1_Pin GPIO_PIN_0
#define PT_CH_1_GPIO_Port GPIOB
#define PT_CH_0_Pin GPIO_PIN_1
#define PT_CH_0_GPIO_Port GPIOB

uint32_t pt_adc_buff[4];
uint16_t ctrl_adc_buff[6];
float pt_data[8];
uint16_t pt_maxes[8] = { 1000, 1450, 1450, 5800, 2000, 2000, 2000, 2000 };
uint32_t pt_mask;

#endif /* INC_DRIVERS_PRESSURE_TRANSDUCER_H_ */
