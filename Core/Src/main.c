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
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "AD7124.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_ANGLE 0
#define MAX_ANGLE 180

#define SERVOMIN  544 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2400 // This is the 'maximum' pulse length count (out of 4096)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ANGLE_TO_US(a) ((uint16_t)((((a) - MIN_ANGLE) * (SERVOMAX - SERVOMIN) / (MAX_ANGLE - MIN_ANGLE)) + SERVOMIN))
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
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[4];

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[4];
uint8_t count = 0;

uint8_t enable_data = 0;

void enable_servo(uint8_t servo_id, uint8_t is_enabled){
	switch(servo_id){
		case 1:
			HAL_GPIO_WritePin(SERV_CTL_1_Pin, SERV_CTL_1_GPIO_Port,is_enabled);
			break;
		case 2:
			HAL_GPIO_WritePin(SERV_CTL_2_Pin, SERV_CTL_2_GPIO_Port,is_enabled);
			break;
		case 3:
			HAL_GPIO_WritePin(SERV_CTL_3_Pin, SERV_CTL_3_GPIO_Port,is_enabled);
			break;
	}
}

void set_servo_angle(int8_t servo_id, uint16_t angle){
	switch(servo_id){
		case 1:
			TIM2->CCR2 = ANGLE_TO_US(angle);
			break;
		case 2:
			TIM2->CCR3 = ANGLE_TO_US(angle);
			break;
		case 3:
			TIM2->CCR4 = ANGLE_TO_US(angle);
			break;
	}
}

void enable_solinoid(uint8_t solinoid_id,uint8_t is_enabled){
	switch(solinoid_id){
		case 0:
			HAL_GPIO_WritePin(SOL_0_Pin, SOL_0_GPIO_Port,is_enabled);
			break;
		case 1:
			HAL_GPIO_WritePin(SOL_1_Pin, SOL_1_GPIO_Port,is_enabled);
			break;
		case 2:
			HAL_GPIO_WritePin(SOL_2_Pin, SOL_2_GPIO_Port,is_enabled);
			break;
		case 3:
			HAL_GPIO_WritePin(SOL_3_Pin, SOL_3_GPIO_Port,is_enabled);
			break;
		case 4:
			HAL_GPIO_WritePin(SOL_4_Pin, SOL_4_GPIO_Port,is_enabled);
			break;
		case 5:
			HAL_GPIO_WritePin(SOL_5_Pin, SOL_5_GPIO_Port,is_enabled);
			break;
		case 6:
			HAL_GPIO_WritePin(SOL_6_Pin, SOL_6_GPIO_Port,is_enabled);
			break;
		case 7:
			HAL_GPIO_WritePin(SOL_7_Pin, SOL_7_GPIO_Port,is_enabled);
			break;
	}
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	if ((RxHeader.StdId = 0x1DA))
		count = RxHeader.DLC;

		switch((RxData[0] << 8 )| RxData[1]){
			// Set servo angle
			case 0xAA:
				packet = RxData[2] << 8 | RxData[3];
				set_servo_angle(packet & 0b11, packet >> 2);
				break;
			// Enable/Disable servo power
			case 0xB0:
				packet = RxData[3];
				enable_servo(packet & 0b11, (packet >> 2) != 0);
				break;
			// Power/Off Solenoid
			case 0xEB:
				enable_solinoid(RxData[2], RxData[3] != 0);
				break;
			// Request Data
			case 0xDA:
				enable_data = RxData[3] != 0;
				break;
			// PCB^2 Error Signal
			case 0xDE:
		}


}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(1000);

  HAL_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_PWM_Start(&htim4,TIM_CHANNEL_4);


  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //activates rx interrupt


  HAL_ADC_Start(&hadc2);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, pt_adc_buff, 4);

  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_IT(&hadc3);
  HAL_ADC_Start_DMA(&hadc3, ctrl_adc_buff, 6);
  HAL_TIM_Base_Start(&htim3);

  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x2CB;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 4;

  TxData[0] = 0xEF;
  TxData[1] = 0xBE;
  TxData[2] = 0xAD;
  TxData[3] = 0xDE;
  if ((HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox))
  			!= HAL_OK) {
  		Error_Handler();
  	}

//  int8_t flag;
//
//  struct ad7124_dev ad7124_device;
//
//  struct ad7124_registerData ad7124_init_regs_default[AD7124_REG_NO] = {
//  	{0x00, 0x00,   1, 2}, /* AD7124_Status */
//  	/*
//  	 * 7:6 - 11 : Full power mode
//  	 * 8 - 1: Internale ref enabled
//  	 * 1:0 - 00 Internal 614.4 clock
//  	 */
//  	{0x01, 0x05C0, 2, 1}, /* AD7124_ADC_Control */
//  	{0x02, 0x0000, 3, 2}, /* AD7124_Data */
//  	{0x03, 0x0000, 3, 1}, /* AD7124_IOCon1 */
//  	{0x04, 0x0000, 2, 1}, /* AD7124_IOCon2 */
//  	{0x05, 0x02,   1, 2}, /* AD7124_ID */
//  	{0x06, 0x0000, 3, 2}, /* AD7124_Error */
//  	{0x07, 0x0000, 3, 1}, /* AD7124_Error_En */
//  	{0x08, 0x00,   1, 2}, /* AD7124_Mclk_Count */
//  	{0x09, 0x0001, 2, 1}, /* AD7124_Channel_0 */
//  	{0x0A, 0x0043, 2, 1}, /* AD7124_Channel_1 */
//  	{0x0B, 0x0043, 2, 1}, /* AD7124_Channel_2 */
//  	{0x0C, 0x00C7, 2, 1}, /* AD7124_Channel_3 */
//  	{0x0D, 0x0085, 2, 1}, /* AD7124_Channel_4 */
//  	{0x0E, 0x014B, 2, 1}, /* AD7124_Channel_5 */
//  	{0x0F, 0x00C7, 2, 1}, /* AD7124_Channel_6 */
//  	{0x10, 0x01CF, 2, 1}, /* AD7124_Channel_7 */
//  	{0x11, 0x0109, 2, 1}, /* AD7124_Channel_8 */
//  	{0x12, 0x0031, 2, 1}, /* AD7124_Channel_9 */
//  	{0x13, 0x014B, 2, 1}, /* AD7124_Channel_10 */
//  	{0x14, 0x0071, 2, 1}, /* AD7124_Channel_11 */
//  	{0x15, 0x018D, 2, 1}, /* AD7124_Channel_12 */
//  	{0x16, 0x00B1, 2, 1}, /* AD7124_Channel_13 */
//  	{0x17, 0x01CF, 2, 1}, /* AD7124_Channel_14 */
//  	{0x18, 0x00F1, 2, 1}, /* AD7124_Channel_15 */
//  	{0x19, 0x0980, 2, 1}, /* AD7124_Config_0 */
//  	{0x1A, 0x0981, 2, 1}, /* AD7124_Config_1 */
//  	{0x1B, 0x0982, 2, 1}, /* AD7124_Config_2 */
//  	{0x1C, 0x0983, 2, 1}, /* AD7124_Config_3 */
//  	{0x1D, 0x0984, 2, 1}, /* AD7124_Config_4 */
//  	{0x1E, 0x0985, 2, 1}, /* AD7124_Config_5 */
//  	{0x1F, 0x0986, 2, 1}, /* AD7124_Config_6 */
//  	{0x20, 0x0987, 2, 1}, /* AD7124_Config_7 */
//  	/*
//  	 * 10:0 - 1 - FS = 1 from filter 0 to 7
//  	 */
//  	{0x21, 0x060001, 3, 1}, /* AD7124_Filter_0 */
//  	{0x22, 0x060001, 3, 1}, /* AD7124_Filter_1 */
//  	{0x23, 0x060001, 3, 1}, /* AD7124_Filter_2 */
//  	{0x24, 0x060001, 3, 1}, /* AD7124_Filter_3 */
//  	{0x25, 0x060001, 3, 1}, /* AD7124_Filter_4 */
//  	{0x26, 0x060001, 3, 1}, /* AD7124_Filter_5 */
//  	{0x27, 0x060001, 3, 1}, /* AD7124_Filter_6 */
//  	{0x28, 0x060001, 3, 1}, /* AD7124_Filter_7 */
//  	{0x29, 0x800000, 3, 1}, /* AD7124_Offset_0 */
//  	{0x2A, 0x800000, 3, 1}, /* AD7124_Offset_1 */
//  	{0x2B, 0x800000, 3, 1}, /* AD7124_Offset_2 */
//  	{0x2C, 0x800000, 3, 1}, /* AD7124_Offset_3 */
//  	{0x2D, 0x800000, 3, 1}, /* AD7124_Offset_4 */
//  	{0x2E, 0x800000, 3, 1}, /* AD7124_Offset_5 */
//  	{0x2F, 0x800000, 3, 1}, /* AD7124_Offset_6 */
//  	{0x30, 0x800000, 3, 1}, /* AD7124_Offset_7 */
//  	{0x31, 0x500000, 3, 1}, /* AD7124_Gain_0 */
//  	{0x32, 0x500000, 3, 1}, /* AD7124_Gain_1 */
//  	{0x33, 0x500000, 3, 1}, /* AD7124_Gain_2 */
//  	{0x34, 0x500000, 3, 1}, /* AD7124_Gain_3 */
//  	{0x35, 0x500000, 3, 1}, /* AD7124_Gain_4 */
//  	{0x36, 0x500000, 3, 1}, /* AD7124_Gain_5 */
//  	{0x37, 0x500000, 3, 1}, /* AD7124_Gain_6 */
//  	{0x38, 0x500000, 3, 1}, /* AD7124_Gain_7 */
//  };
//


//  flag = ad7124_init(&ad7124_device, &hspi2, GPIOD, GPIO_PIN_10, ID_AD7124_8, &ad7124_init_regs_default);



//  if (flag < 0)
//  		return flag;


  HAL_Delay(100);

//  ad7124_noCheckReadRegister(&ad7124_device, &ad7124_device.regs[0x06]);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (!enable_data)
    HAL_Delay(500);
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if ((HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox))
	  	  				!= HAL_OK) {
	  	  			Error_Handler();
	  	  		}
	  	  		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
	  	  		HAL_Delay(10);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
