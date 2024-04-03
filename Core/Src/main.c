/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "fmpi2c.h"
#include "i2c.h"
#include "quadspi.h"
#include "sai.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stm32f4xx_hal.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "AD7124.h"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//static CAN_TxHeaderTypeDef TxHeader;
//static uint32_t TxMailbox;
//static uint8_t TxData[5];

int32_t value[16];
float voltage[16];

//static CAN_RxHeaderTypeDef RxHeader;
//static uint8_t RxData[4];
//static uint8_t count = 0;

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
//	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
//	if ((RxHeader.StdId = 0x1DA)) {
//		count = RxHeader.DLC;
//
//		switch (RxData[0]) {
//		// Set servo angle
//		case 0xAA:
//			int16_t packet = RxData[2] << 8 | RxData[3];
//			set_servo_angle(packet & 0b11, packet >> 2);
//			break;
//			// Enable/Disable servo power
//		case 0xB0:
//			packet = RxData[3];
//			enable_servo(packet & 0b11, (packet >> 2) != 0);
//			break;
//			// Power/Off Solenoid
//		case 0xEB:
//			enable_solinoid(RxData[2], RxData[3] != 0);
//			break;
//			// Send Data
//		case 0xDA:
//			for (int i = 0; i < 22; i++) {
//				TxData[0] = i;
//				TxData[1] = 0xBA;
//				TxData[2] = 0xDA;
//				TxData[3] = 0x55;
//				TxData[4] = 0x69;
//				if ((HAL_CAN_AddTxMessage(hcan1, &TxHeader, TxData, &TxMailbox))
//						!= HAL_OK)
//					return;
//			}
//			break;
//			// PCB^2 Error Signal
//		case 0xDE:
//		}
//
//	}
//}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	/* FPU settings ------------------------------------------------------------*/

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_CAN1_Init();
	MX_FMPI2C1_Init();
	MX_I2C3_Init();
	MX_QUADSPI_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_TIM4_Init();
	MX_SPI4_Init();
	MX_TIM3_Init();
	MX_ADC3_Init();
	MX_SAI2_Init();
	/* USER CODE BEGIN 2 */
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
//	HAL_Delay(1000);
//
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
//
//	HAL_CAN_Start(&hcan1);
//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //activates rx interrupt
//
//	//HAL_ADC_Start(&hadc2);
//	HAL_ADC_Start(&hadc2);
//	HAL_ADCEx_MultiModeStart_DMA(&hadc1, pt_adc_buff, 4);
//	//HAL_ADC_Start_DMA(&hadc1, adc_buff_ptr, 8);
//	HAL_ADC_Start_IT(&hadc1);
//	HAL_ADC_Start_IT(&hadc3);
//	HAL_ADC_Start_DMA(&hadc3, ctrl_adc_buff, 6);
//	HAL_TIM_Base_Start(&htim3);
//
//	TxHeader.IDE = CAN_ID_STD;
//	TxHeader.StdId = 0x2CB;
//	TxHeader.RTR = CAN_RTR_DATA;
//	TxHeader.DLC = 5;
//
//	TxData[0] = 0xEF;
//	TxData[1] = 0xBE;
//	TxData[2] = 0xAD;
//	TxData[3] = 0xDE;
//	if ((HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox))
//			!= HAL_OK) {
//		Error_Handler();
//	}
//
	int8_t flag;

	struct ad7124_dev ad7124_device;

	flag = ad7124_init(&ad7124_device, &hspi2, GPIOD, GPIO_PIN_10);

	if (flag != 0)
		return flag;
//
//	HAL_Delay(100);
//
	ad7124_noCheckReadRegister(&ad7124_device, &ad7124_device.regs[0x06]);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		//ad7124reading();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		value[0] = ad7124_adcRead(&ad7124_device, 0);
		value[1] = ad7124_adcRead(&ad7124_device, 1);
		value[2] = ad7124_adcRead(&ad7124_device, 2);
		value[3] = ad7124_adcRead(&ad7124_device, 3);
		value[4] = ad7124_adcRead(&ad7124_device, 4);
		value[5] = ad7124_adcRead(&ad7124_device, 5);
		value[6] = ad7124_adcRead(&ad7124_device, 6);
		value[7] = ad7124_adcRead(&ad7124_device, 7);
		value[8] = ad7124_adcRead(&ad7124_device, 8);
		value[9] = ad7124_adcRead(&ad7124_device, 9);
		value[10] = ad7124_adcRead(&ad7124_device, 10);
		value[11] = ad7124_adcRead(&ad7124_device, 11);
		value[12] = ad7124_adcRead(&ad7124_device, 12);
		value[13] = ad7124_adcRead(&ad7124_device, 13);
		value[14] = ad7124_adcRead(&ad7124_device, 14);
		value[15] = ad7124_adcRead(&ad7124_device, 15);

		HAL_Delay(50);
		voltage[0] = ad7124_toVoltage(value[0], 1, 3.3, 1);
		voltage[1] = ad7124_toVoltage(value[1], 1, 3.3, 1);
		voltage[2] = ad7124_toVoltage(value[2], 1, 3.3, 1);
		voltage[3] = ad7124_toVoltage(value[3], 1, 3.3, 1);
		voltage[4] = ad7124_toVoltage(value[4], 1, 3.3, 1);
		voltage[5] = ad7124_toVoltage(value[5], 1, 3.3, 1);
		voltage[6] = ad7124_toVoltage(value[6], 1, 3.3, 1);
		voltage[7] = ad7124_toVoltage(value[7], 1, 3.3, 1);
		voltage[8] = ad7124_toVoltage(value[8], 1, 3.3, 1);
		voltage[9] = ad7124_toVoltage(value[9], 1, 3.3, 1);
		voltage[10] = ad7124_toVoltage(value[10], 1, 3.3, 1);
		voltage[11] = ad7124_toVoltage(value[11], 1, 3.3, 1);
		voltage[12] = ad7124_toVoltage(value[12], 1, 3.3, 1);
		voltage[13] = ad7124_toVoltage(value[13], 1, 3.3, 1);
		voltage[14] = ad7124_toVoltage(value[14], 1, 3.3, 1);
		voltage[15] = ad7124_toVoltage(value[15], 1, 3.3, 1);
		HAL_Delay(50);

//		for (i = 0; i < 22; i++) {
//			TxData[0] = i;
//			TxData[1] = 0xBA;
//			TxData[2] = 0xDA;
//			TxData[3] = 0x55;
//			TxData[4] = 0x69;
//			if ((HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox))
//					!= HAL_OK)
//				return 0;
//			//	  				}
//			HAL_Delay(500);
//			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
//		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 224;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	if (hadc->Instance == ADC1) {
//		for (int i = 0; i < 8; i++) {
//			if (i % 2)
//				pt_data[i] = (float) ((3.3f)
//						* ((pt_adc_buff[i / 2] & 0x0FFF0000) >> 16) / 4096);
//			else
//				pt_data[i] = (float) ((3.3f) * (pt_adc_buff[i / 2] & 0x00000FFF)
//						/ 4096);
//
//			pt_data[i] = pt_data[i] - 0.6;
//			pt_data[i] *= (float) (pt_maxes[i] / (2.4f));
//		}
//		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
//	}
//}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
