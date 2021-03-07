/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LORA_RST_Pin GPIO_PIN_2
#define LORA_RST_GPIO_Port GPIOE
#define LED_ERROR_Pin GPIO_PIN_3
#define LED_ERROR_GPIO_Port GPIOE
#define SD_DETECT_Pin GPIO_PIN_4
#define SD_DETECT_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOF
#define LED_WARNING_Pin GPIO_PIN_7
#define LED_WARNING_GPIO_Port GPIOF
#define LED_CANB_Pin GPIO_PIN_8
#define LED_CANB_GPIO_Port GPIOF
#define LED_CANA_Pin GPIO_PIN_9
#define LED_CANA_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define TORQUE_ADC_Pin GPIO_PIN_0
#define TORQUE_ADC_GPIO_Port GPIOC
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define LOADCELL_ADC_Pin GPIO_PIN_3
#define LOADCELL_ADC_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define MAST_CLOCK_Pin GPIO_PIN_2
#define MAST_CLOCK_GPIO_Port GPIOB
#define LORA_EN_Pin GPIO_PIN_12
#define LORA_EN_GPIO_Port GPIOF
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOF
#define ROTOR_RPM2_Pin GPIO_PIN_10
#define ROTOR_RPM2_GPIO_Port GPIOE
#define WHEEL_RPM_Pin GPIO_PIN_11
#define WHEEL_RPM_GPIO_Port GPIOE
#define ROTOR_RPM_Pin GPIO_PIN_12
#define ROTOR_RPM_GPIO_Port GPIOE
#define WHEEL_RPM2_Pin GPIO_PIN_13
#define WHEEL_RPM2_GPIO_Port GPIOE
#define PITCH_CLOCK_Pin GPIO_PIN_13
#define PITCH_CLOCK_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define LORA_INT_Pin GPIO_PIN_11
#define LORA_INT_GPIO_Port GPIOD
#define MAST_DATA_Pin GPIO_PIN_13
#define MAST_DATA_GPIO_Port GPIOD
#define LIMIT2_Pin GPIO_PIN_2
#define LIMIT2_GPIO_Port GPIOG
#define LIMIT1_Pin GPIO_PIN_3
#define LIMIT1_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define PITCH_DATA_Pin GPIO_PIN_4
#define PITCH_DATA_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
