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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM3_ARR_VALUE 3839
#define TIM4_ARR_VALUE 3839
#define TIM8_PSC_VALUE 959
#define TIM8_ARR_VALUE 399
#define TIM6_PSC_VALUE 959
#define TIM6_ARR_VALUE 999
#define TIM1_PSC_VALUE 959
#define TIM1_ARR_VALUE 1999
#define GPIO_OUT_SPI_CS_SDCARD_Pin GPIO_PIN_3
#define GPIO_OUT_SPI_CS_SDCARD_GPIO_Port GPIOE
#define GPIO_OUT_SPI_CS_LCD_Pin GPIO_PIN_4
#define GPIO_OUT_SPI_CS_LCD_GPIO_Port GPIOE
#define TIM9_CH1_USER_LED1_Pin GPIO_PIN_5
#define TIM9_CH1_USER_LED1_GPIO_Port GPIOE
#define TIM9_CH2_USER_LED2_Pin GPIO_PIN_6
#define TIM9_CH2_USER_LED2_GPIO_Port GPIOE
#define GPIO_EXTI2_PROXY_TOF_SENS_IRQ_Pin GPIO_PIN_2
#define GPIO_EXTI2_PROXY_TOF_SENS_IRQ_GPIO_Port GPIOF
#define GPIO_EXTI3_IMU_IRQ_Pin GPIO_PIN_3
#define GPIO_EXTI3_IMU_IRQ_GPIO_Port GPIOF
#define GPIO_EXTI4_KPAD_IRQ_Pin GPIO_PIN_4
#define GPIO_EXTI4_KPAD_IRQ_GPIO_Port GPIOF
#define GPIO_EXTI8_USER_BUT1_IRQ_Pin GPIO_PIN_8
#define GPIO_EXTI8_USER_BUT1_IRQ_GPIO_Port GPIOF
#define GPIO_EXTI9_USER_BUT2_IRQ_Pin GPIO_PIN_9
#define GPIO_EXTI9_USER_BUT2_IRQ_GPIO_Port GPIOF
#define GPIO_EXTI10_BUMP1_IRQ_Pin GPIO_PIN_10
#define GPIO_EXTI10_BUMP1_IRQ_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define TIM5_CH1_BUZZ_Pin GPIO_PIN_0
#define TIM5_CH1_BUZZ_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define ADC1_IN3_IR_DIST_SENS_Pin GPIO_PIN_3
#define ADC1_IN3_IR_DIST_SENS_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define GPIO_EXTI11_BUMP2_IRQ_Pin GPIO_PIN_11
#define GPIO_EXTI11_BUMP2_IRQ_GPIO_Port GPIOF
#define GPIO_EXTI12_BUMP3_IRQ_Pin GPIO_PIN_12
#define GPIO_EXTI12_BUMP3_IRQ_GPIO_Port GPIOF
#define GPIO_EXTI13_BUMP4_IRQ_Pin GPIO_PIN_13
#define GPIO_EXTI13_BUMP4_IRQ_GPIO_Port GPIOF
#define TIM1_CH1_SERVO1_Pin GPIO_PIN_9
#define TIM1_CH1_SERVO1_GPIO_Port GPIOE
#define TIM1_CH2_SERVO2_Pin GPIO_PIN_11
#define TIM1_CH2_SERVO2_GPIO_Port GPIOE
#define TIM1_CH3_SERVO3_Pin GPIO_PIN_13
#define TIM1_CH3_SERVO3_GPIO_Port GPIOE
#define TIM1_CH4_SERVO4_Pin GPIO_PIN_14
#define TIM1_CH4_SERVO4_GPIO_Port GPIOE
#define TIM2_CH3_HCSR04_ECHO_Pin GPIO_PIN_10
#define TIM2_CH3_HCSR04_ECHO_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define TIM4_CH1_ENC2A_Pin GPIO_PIN_12
#define TIM4_CH1_ENC2A_GPIO_Port GPIOD
#define TIM4_CH2_ENC2B_Pin GPIO_PIN_13
#define TIM4_CH2_ENC2B_GPIO_Port GPIOD
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define TIM8_CH1_MOT1A_Pin GPIO_PIN_6
#define TIM8_CH1_MOT1A_GPIO_Port GPIOC
#define TIM8_CH2_MOT1B_Pin GPIO_PIN_7
#define TIM8_CH2_MOT1B_GPIO_Port GPIOC
#define TIM8_CH3_MOT2A_Pin GPIO_PIN_8
#define TIM8_CH3_MOT2A_GPIO_Port GPIOC
#define TIM8_CH4_MOT2B_Pin GPIO_PIN_9
#define TIM8_CH4_MOT2B_GPIO_Port GPIOC
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
#define TIM2_CH1_HCSR04_TRIG_Pin GPIO_PIN_15
#define TIM2_CH1_HCSR04_TRIG_GPIO_Port GPIOA
#define UART4_TX_LCD_Pin GPIO_PIN_10
#define UART4_TX_LCD_GPIO_Port GPIOC
#define UART4_RX_LCD_Pin GPIO_PIN_11
#define UART4_RX_LCD_GPIO_Port GPIOC
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TIM3_CH1_ENC1A_Pin GPIO_PIN_4
#define TIM3_CH1_ENC1A_GPIO_Port GPIOB
#define TIM3_CH2_ENC1B_Pin GPIO_PIN_5
#define TIM3_CH2_ENC1B_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
