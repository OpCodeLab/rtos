/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "semphr.h"

SemaphoreHandle_t xUartMutex;   // in main(), before the scheduler starts: xUartMutex = xSemaphoreCreateMutex();
extern UART_HandleTypeDef huart2;

int _write(int file, char *ptr, int len)
{
  (void)file;
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    xSemaphoreTake(xUartMutex, portMAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    xSemaphoreGive(xUartMutex);
  return len;
}
