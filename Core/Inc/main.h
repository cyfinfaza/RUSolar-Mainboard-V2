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
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CON_BAT_G_Pin GPIO_PIN_2
#define CON_BAT_G_GPIO_Port GPIOE
#define CON_SC_G_Pin GPIO_PIN_3
#define CON_SC_G_GPIO_Port GPIOE
#define CON_EC_G_Pin GPIO_PIN_4
#define CON_EC_G_GPIO_Port GPIOE
#define SIG_G_L_Pin GPIO_PIN_5
#define SIG_G_L_GPIO_Port GPIOE
#define SIG_G_R_Pin GPIO_PIN_6
#define SIG_G_R_GPIO_Port GPIOE
#define USR_BTN_1_Pin GPIO_PIN_13
#define USR_BTN_1_GPIO_Port GPIOC
#define USR_LED_1_Pin GPIO_PIN_8
#define USR_LED_1_GPIO_Port GPIOF
#define USR_LED_2_Pin GPIO_PIN_9
#define USR_LED_2_GPIO_Port GPIOF
#define USR_BTN_2_Pin GPIO_PIN_10
#define USR_BTN_2_GPIO_Port GPIOF
#define HRN_G_Pin GPIO_PIN_0
#define HRN_G_GPIO_Port GPIOC
#define BMS_IO1_Pin GPIO_PIN_1
#define BMS_IO1_GPIO_Port GPIOC
#define BMS_IO2_Pin GPIO_PIN_2
#define BMS_IO2_GPIO_Port GPIOC
#define BMS_IO3_Pin GPIO_PIN_3
#define BMS_IO3_GPIO_Port GPIOC
#define SWL_FR_G_Pin GPIO_PIN_1
#define SWL_FR_G_GPIO_Port GPIOA
#define BMS_DCL_Pin GPIO_PIN_2
#define BMS_DCL_GPIO_Port GPIOA
#define BMS_SOC_Pin GPIO_PIN_3
#define BMS_SOC_GPIO_Port GPIOA
#define BMS_CCL_Pin GPIO_PIN_4
#define BMS_CCL_GPIO_Port GPIOA
#define IO1_ADC_Pin GPIO_PIN_5
#define IO1_ADC_GPIO_Port GPIOA
#define IO2_ADC_Pin GPIO_PIN_6
#define IO2_ADC_GPIO_Port GPIOA
#define IO3_ADC_Pin GPIO_PIN_7
#define IO3_ADC_GPIO_Port GPIOA
#define BMS_CH_EN_Pin GPIO_PIN_4
#define BMS_CH_EN_GPIO_Port GPIOC
#define BMS_DCH_EN_Pin GPIO_PIN_5
#define BMS_DCH_EN_GPIO_Port GPIOC
#define IO4_ADC_Pin GPIO_PIN_0
#define IO4_ADC_GPIO_Port GPIOB
#define BPSFL_G_Pin GPIO_PIN_1
#define BPSFL_G_GPIO_Port GPIOB
#define SW_BRK_Pin GPIO_PIN_2
#define SW_BRK_GPIO_Port GPIOB
#define SW_PRK_Pin GPIO_PIN_11
#define SW_PRK_GPIO_Port GPIOF
#define SW_HRN_Pin GPIO_PIN_12
#define SW_HRN_GPIO_Port GPIOF
#define SW_HL_Pin GPIO_PIN_13
#define SW_HL_GPIO_Port GPIOF
#define SW_TL_Pin GPIO_PIN_14
#define SW_TL_GPIO_Port GPIOF
#define SW_TR_Pin GPIO_PIN_15
#define SW_TR_GPIO_Port GPIOF
#define SW_SCE_Pin GPIO_PIN_0
#define SW_SCE_GPIO_Port GPIOG
#define SW_HAZ_Pin GPIO_PIN_1
#define SW_HAZ_GPIO_Port GPIOG
#define BAT_INT_SENSE_Pin GPIO_PIN_7
#define BAT_INT_SENSE_GPIO_Port GPIOE
#define SW_BAT_Pin GPIO_PIN_8
#define SW_BAT_GPIO_Port GPIOE
#define BRKL_G_L_Pin GPIO_PIN_9
#define BRKL_G_L_GPIO_Port GPIOE
#define SW_MO_PWR_Pin GPIO_PIN_10
#define SW_MO_PWR_GPIO_Port GPIOE
#define BRKL_G_R_Pin GPIO_PIN_11
#define BRKL_G_R_GPIO_Port GPIOE
#define DTRL_G_L_Pin GPIO_PIN_13
#define DTRL_G_L_GPIO_Port GPIOE
#define DTRL_G_R_Pin GPIO_PIN_14
#define DTRL_G_R_GPIO_Port GPIOE
#define SWL_FR_B_Pin GPIO_PIN_10
#define SWL_FR_B_GPIO_Port GPIOB
#define HMCBL_G_Pin GPIO_PIN_11
#define HMCBL_G_GPIO_Port GPIOB
#define SWL_SCE_R_Pin GPIO_PIN_12
#define SWL_SCE_R_GPIO_Port GPIOD
#define SWL_SCE_G_Pin GPIO_PIN_13
#define SWL_SCE_G_GPIO_Port GPIOD
#define SWL_SCE_B_Pin GPIO_PIN_14
#define SWL_SCE_B_GPIO_Port GPIOD
#define SWL_HAZ_Pin GPIO_PIN_15
#define SWL_HAZ_GPIO_Port GPIOD
#define SWL_PE_R_Pin GPIO_PIN_6
#define SWL_PE_R_GPIO_Port GPIOC
#define SWL_EXTI_Pin GPIO_PIN_7
#define SWL_EXTI_GPIO_Port GPIOC
#define SWL_PE_B_Pin GPIO_PIN_8
#define SWL_PE_B_GPIO_Port GPIOC
#define SWL_FR_R_Pin GPIO_PIN_15
#define SWL_FR_R_GPIO_Port GPIOA
#define CAN_RX_Pin GPIO_PIN_0
#define CAN_RX_GPIO_Port GPIOD
#define CAN_TX_Pin GPIO_PIN_1
#define CAN_TX_GPIO_Port GPIOD
#define MO_RPM_Pin GPIO_PIN_10
#define MO_RPM_GPIO_Port GPIOG
#define MO_PE_Pin GPIO_PIN_11
#define MO_PE_GPIO_Port GPIOG
#define MO_FR_Pin GPIO_PIN_12
#define MO_FR_GPIO_Port GPIOG
#define MO_PWR_G_Pin GPIO_PIN_13
#define MO_PWR_G_GPIO_Port GPIOG
#define SWL_PE_G_Pin GPIO_PIN_5
#define SWL_PE_G_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
