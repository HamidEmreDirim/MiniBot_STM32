/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

extern volatile uint32_t encoder_m1_count;

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
#define LORA_NRST_Pin GPIO_PIN_2
#define LORA_NRST_GPIO_Port GPIOE
#define CAM_LED_ON_OFF_Pin GPIO_PIN_3
#define CAM_LED_ON_OFF_GPIO_Port GPIOE
#define NEON_LED2_ON_OFF_Pin GPIO_PIN_4
#define NEON_LED2_ON_OFF_GPIO_Port GPIOE
#define NEON_LED1_ON_OFF_Pin GPIO_PIN_6
#define NEON_LED1_ON_OFF_GPIO_Port GPIOE
#define FAN_ON_OFF_Pin GPIO_PIN_13
#define FAN_ON_OFF_GPIO_Port GPIOC
#define ADC1_IN12_Pin GPIO_PIN_2
#define ADC1_IN12_GPIO_Port GPIOC
#define EN_3V3_Pin GPIO_PIN_3
#define EN_3V3_GPIO_Port GPIOC
#define LORA_RX_STM_TX4_Pin GPIO_PIN_0
#define LORA_RX_STM_TX4_GPIO_Port GPIOA
#define LORA_TX_STM_RX4_Pin GPIO_PIN_1
#define LORA_TX_STM_RX4_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define M2_L_PWM_Pin GPIO_PIN_6
#define M2_L_PWM_GPIO_Port GPIOA
#define M2_R_PWM_Pin GPIO_PIN_7
#define M2_R_PWM_GPIO_Port GPIOA
#define M1_ENCA_EXT4_Pin GPIO_PIN_4
#define M1_ENCA_EXT4_GPIO_Port GPIOC
#define M1_ENCA_EXT4_EXTI_IRQn EXTI4_IRQn
#define M1_ENCB_EXT5_Pin GPIO_PIN_5
#define M1_ENCB_EXT5_GPIO_Port GPIOC
#define M1_ENCB_EXT5_EXTI_IRQn EXTI9_5_IRQn
#define M1_R_PWM_Pin GPIO_PIN_0
#define M1_R_PWM_GPIO_Port GPIOB
#define M1_L_PWM_Pin GPIO_PIN_1
#define M1_L_PWM_GPIO_Port GPIOB
#define M2_ENCB_EXT7_Pin GPIO_PIN_7
#define M2_ENCB_EXT7_GPIO_Port GPIOE
#define M2_ENCB_EXT7_EXTI_IRQn EXTI9_5_IRQn
#define M2_ENCA_EXT8_Pin GPIO_PIN_8
#define M2_ENCA_EXT8_GPIO_Port GPIOE
#define M2_ENCA_EXT8_EXTI_IRQn EXTI9_5_IRQn
#define M3_L_PWM_Pin GPIO_PIN_9
#define M3_L_PWM_GPIO_Port GPIOE
#define M3_ENCA_EXT10_Pin GPIO_PIN_10
#define M3_ENCA_EXT10_GPIO_Port GPIOE
#define M3_ENCA_EXT10_EXTI_IRQn EXTI15_10_IRQn
#define M3_R_PWM_Pin GPIO_PIN_11
#define M3_R_PWM_GPIO_Port GPIOE
#define M3_ENCN_EXT12_Pin GPIO_PIN_12
#define M3_ENCN_EXT12_GPIO_Port GPIOE
#define M3_ENCN_EXT12_EXTI_IRQn EXTI15_10_IRQn
#define M4_R_PWM_Pin GPIO_PIN_13
#define M4_R_PWM_GPIO_Port GPIOE
#define M4_L_PWM_Pin GPIO_PIN_14
#define M4_L_PWM_GPIO_Port GPIOE
#define TH_NRST_Pin GPIO_PIN_15
#define TH_NRST_GPIO_Port GPIOE
#define TH_SCL_SCL2_Pin GPIO_PIN_10
#define TH_SCL_SCL2_GPIO_Port GPIOB
#define TH_SDA_SDA2_Pin GPIO_PIN_11
#define TH_SDA_SDA2_GPIO_Port GPIOB
#define M4_L_EN_Pin GPIO_PIN_12
#define M4_L_EN_GPIO_Port GPIOB
#define M3_R_EN_Pin GPIO_PIN_13
#define M3_R_EN_GPIO_Port GPIOB
#define NEON_LED1_PWM_Pin GPIO_PIN_14
#define NEON_LED1_PWM_GPIO_Port GPIOB
#define NEON_LED2_PWM_Pin GPIO_PIN_15
#define NEON_LED2_PWM_GPIO_Port GPIOB
#define GPS_RX_STM_TX3_Pin GPIO_PIN_8
#define GPS_RX_STM_TX3_GPIO_Port GPIOD
#define GPS_TX_STM_RX3_Pin GPIO_PIN_9
#define GPS_TX_STM_RX3_GPIO_Port GPIOD
#define M3_L_EN_Pin GPIO_PIN_10
#define M3_L_EN_GPIO_Port GPIOD
#define M4_R_EN_Pin GPIO_PIN_11
#define M4_R_EN_GPIO_Port GPIOD
#define M1_L_EN_Pin GPIO_PIN_12
#define M1_L_EN_GPIO_Port GPIOD
#define M2_R_EN_Pin GPIO_PIN_13
#define M2_R_EN_GPIO_Port GPIOD
#define M1_R_EN_Pin GPIO_PIN_14
#define M1_R_EN_GPIO_Port GPIOD
#define M2_L_EN_Pin GPIO_PIN_15
#define M2_L_EN_GPIO_Port GPIOD
#define ESP_RX_STM_TX6_Pin GPIO_PIN_6
#define ESP_RX_STM_TX6_GPIO_Port GPIOC
#define ESP_TX_STM_RX6_Pin GPIO_PIN_7
#define ESP_TX_STM_RX6_GPIO_Port GPIOC
#define INA_SDA_SDA3_Pin GPIO_PIN_9
#define INA_SDA_SDA3_GPIO_Port GPIOC
#define INA_SCL_SCL3_Pin GPIO_PIN_8
#define INA_SCL_SCL3_GPIO_Port GPIOA
#define M4_ENCBA_EXT11_Pin GPIO_PIN_11
#define M4_ENCBA_EXT11_GPIO_Port GPIOC
#define M4_ENCBA_EXT11_EXTI_IRQn EXTI15_10_IRQn
#define M4_ENCB_EXT0_Pin GPIO_PIN_0
#define M4_ENCB_EXT0_GPIO_Port GPIOD
#define M4_ENCB_EXT0_EXTI_IRQn EXTI0_IRQn
#define BNO_NRST_Pin GPIO_PIN_1
#define BNO_NRST_GPIO_Port GPIOD
#define LORA_STATUS_Pin GPIO_PIN_3
#define LORA_STATUS_GPIO_Port GPIOD
#define LORA_STATUS_EXTI_IRQn EXTI3_IRQn
#define GPS_PPS_Pin GPIO_PIN_6
#define GPS_PPS_GPIO_Port GPIOD
#define GPS_PPS_EXTI_IRQn EXTI9_5_IRQn
#define IMU_SCL_SCL1_Pin GPIO_PIN_6
#define IMU_SCL_SCL1_GPIO_Port GPIOB
#define IMU_SDA_SDA1_Pin GPIO_PIN_7
#define IMU_SDA_SDA1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
