/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#define SERV_1_Pin GPIO_PIN_0
#define SERV_1_GPIO_Port GPIOA
#define SERV_2_Pin GPIO_PIN_1
#define SERV_2_GPIO_Port GPIOA
#define SERV_3_Pin GPIO_PIN_2
#define SERV_3_GPIO_Port GPIOA
#define SPI4_CS2_Pin GPIO_PIN_3
#define SPI4_CS2_GPIO_Port GPIOA
#define PT_CH_7_Pin GPIO_PIN_4
#define PT_CH_7_GPIO_Port GPIOA
#define PT_CH_6_Pin GPIO_PIN_5
#define PT_CH_6_GPIO_Port GPIOA
#define PT_CH_5_Pin GPIO_PIN_6
#define PT_CH_5_GPIO_Port GPIOA
#define PT_CH_4_Pin GPIO_PIN_7
#define PT_CH_4_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWDCLK_Pin GPIO_PIN_14
#define SWDCLK_GPIO_Port GPIOA
#define PT_CH_1_Pin GPIO_PIN_0
#define PT_CH_1_GPIO_Port GPIOB
#define PT_CH_0_Pin GPIO_PIN_1
#define PT_CH_0_GPIO_Port GPIOB
#define QSPI_CLK_Pin GPIO_PIN_2
#define QSPI_CLK_GPIO_Port GPIOB
#define SPI1_SCK_Pin GPIO_PIN_3
#define SPI1_SCK_GPIO_Port GPIOB
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_GPIO_Port GPIOB
#define PWM_1_Pin GPIO_PIN_7
#define PWM_1_GPIO_Port GPIOB
#define PWM_2_Pin GPIO_PIN_8
#define PWM_2_GPIO_Port GPIOB
#define PWM_3_Pin GPIO_PIN_9
#define PWM_3_GPIO_Port GPIOB
#define SPI4_CS1_Pin GPIO_PIN_10
#define SPI4_CS1_GPIO_Port GPIOB
#define SPI2_CS3_Pin GPIO_PIN_12
#define SPI2_CS3_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define SOL_0_Pin GPIO_PIN_0
#define SOL_0_GPIO_Port GPIOC
#define SOL_1_Pin GPIO_PIN_1
#define SOL_1_GPIO_Port GPIOC
#define SOL_2_Pin GPIO_PIN_2
#define SOL_2_GPIO_Port GPIOC
#define PT_CH_3_Pin GPIO_PIN_4
#define PT_CH_3_GPIO_Port GPIOC
#define PT_CH_2_Pin GPIO_PIN_5
#define PT_CH_2_GPIO_Port GPIOC
#define SPI1_CS0_Pin GPIO_PIN_6
#define SPI1_CS0_GPIO_Port GPIOC
#define SPI1_CS1_Pin GPIO_PIN_7
#define SPI1_CS1_GPIO_Port GPIOC
#define QSPI_NCS_Pin GPIO_PIN_9
#define QSPI_NCS_GPIO_Port GPIOC
#define WS2812_Pin GPIO_PIN_2
#define WS2812_GPIO_Port GPIOE
#define SOL_GPIO_2_Pin GPIO_PIN_3
#define SOL_GPIO_2_GPIO_Port GPIOE
#define SOL_GPIO_1_Pin GPIO_PIN_4
#define SOL_GPIO_1_GPIO_Port GPIOE
#define SOL_GPIO_0_Pin GPIO_PIN_5
#define SOL_GPIO_0_GPIO_Port GPIOE
#define PWRTR_Pin GPIO_PIN_6
#define PWRTR_GPIO_Port GPIOE
#define QSPI_IO_0_Pin GPIO_PIN_7
#define QSPI_IO_0_GPIO_Port GPIOE
#define QSPI_IO_1_Pin GPIO_PIN_8
#define QSPI_IO_1_GPIO_Port GPIOE
#define QSPI_IO_2_Pin GPIO_PIN_9
#define QSPI_IO_2_GPIO_Port GPIOE
#define QSPI_IO_3_Pin GPIO_PIN_10
#define QSPI_IO_3_GPIO_Port GPIOE
#define SPI4_SCK_Pin GPIO_PIN_12
#define SPI4_SCK_GPIO_Port GPIOE
#define SPI4_MISO_Pin GPIO_PIN_13
#define SPI4_MISO_GPIO_Port GPIOE
#define SPI4_MOSI_Pin GPIO_PIN_14
#define SPI4_MOSI_GPIO_Port GPIOE
#define SPI4_CS0_Pin GPIO_PIN_15
#define SPI4_CS0_GPIO_Port GPIOE
#define CAN_RX_Pin GPIO_PIN_0
#define CAN_RX_GPIO_Port GPIOD
#define CAN_TX_Pin GPIO_PIN_1
#define CAN_TX_GPIO_Port GPIOD
#define SERV_CTL_1_Pin GPIO_PIN_7
#define SERV_CTL_1_GPIO_Port GPIOD
#define SERV_CTL_2_Pin GPIO_PIN_8
#define SERV_CTL_2_GPIO_Port GPIOD
#define SERV_CTL_3_Pin GPIO_PIN_9
#define SERV_CTL_3_GPIO_Port GPIOD
#define SPI2_CS0_Pin GPIO_PIN_10
#define SPI2_CS0_Pin_GPIO_Port GPIOD
#define SCL_1_Pin GPIO_PIN_12
#define SCL_1_GPIO_Port GPIOD
#define SDA_1_Pin GPIO_PIN_13
#define SDA_1_GPIO_Port GPIOD



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
