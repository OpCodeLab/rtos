/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
//#define APK1
//#define APK2
//#define APK3
//#define APK4
//#define APK5
//#define APK6
//#define APK7
//#define APK7_EXE   /* student template of APK7 */
//#define APK8       /* functional safety: NORMAL / DEGRADED / SAFE / RESET */
//#define APK9       /* simple safety: NORMAL / SAFE */
//#define APK10      /* APK9 + test driven development (tests/apk10) */
//#define APK11      /* signal processing: UART samples, double buffer, FFT task */
//#define APK12      /* APK11 simplified: double buffer + CMSIS-DSP FFT (arm_rfft_fast_f32) */
#define APK14      /* APK12 + sensor AutoTest at start-up, SAFE mode, SupervisorTask */
//#define APK15      /* APK12 with two sensors: StoreSample dispatch, FftSensor1 + FftSensor2 */

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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
