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
#include "queue.h"
#include "semphr.h"
#include "log.h"

#ifdef APK1

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void vTask1( void * pvParameters );
void vTask2( void * pvParameters );
void vTask3( void * pvParameters );

/* USER CODE END PFP */
TaskHandle_t xTask1;
TaskHandle_t xTask2;
TaskHandle_t xTask3;
QueueHandle_t xQueue_UART;



/**
  * @brief  The application entry point.
  * @retval int
  */
void run_apk(void)
{

  /* Create a queue capable of containing 10 unsigned long values. */
  xQueue_UART = xQueueCreate( 10, sizeof( uint32_t ) );

  /* Create Task1. */
  xTaskCreate(vTask1,  "Task1",200,NULL,3,&xTask1);     /* Used to pass out the created task's handle. */
  /*Create Task2*/
  xTaskCreate(vTask2,  "Task2",200,NULL,3,&xTask2);
  /*Create Task3*/
  xTaskCreate(vTask3,  "Task3",200,NULL,3,&xTask3);

  LOG_INFO(COLOR_WHITE," RTOS LAB STARTING-----------------");

  vTaskStartScheduler();

  while (1)
  {

  }
}



uint32_t errTX;
uint32_t Producer_Period=500;
uint32_t Consumer_Period=1000;
/*
 * TASK1
 */
void vTask1( void * pvParameters )
{
	uint32_t MyVar=0;
    uint32_t tick=0;

    for(;;) // infinit loop
    {
        tick++;
           MyVar++;
           LOG_INFO(COLOR_YELLOW  ,"<<PRODUCER>> Execution Loop Number %d", (int)tick);

           HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

           if( xQueueSend( xQueue_UART, ( void * ) &MyVar, 100 ) != pdPASS )
           {
               errTX++;
               LOG_ERROR("<<PRODUCER>> : Send Queue Failed = %d", (int)errTX);
           }
           vTaskDelay(Producer_Period);
   }
}

uint32_t Err_QueuEmpty;

void vTask2( void * pvParameters )
{
	uint32_t counter=0;
    uint32_t data_Rx;
    uint32_t tick=0;
    for(;;)
    {
    	tick++;
    	        LOG_INFO(COLOR_CYAN    ,"<<CONSUMER>> Execution Loop Number %d", (int)tick);

    	        if( xQueueReceive(xQueue_UART, (void *) &data_Rx, portMAX_DELAY) != pdPASS )
    	        {
    	            Err_QueuEmpty++;
    	            LOG_ERROR("<<CONSUMER>> : Queue Empty %d", (int)Err_QueuEmpty);
    	        }
    	        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    	        counter++;

    	        vTaskDelay(Consumer_Period);


    }
}
void vTask3( void * pvParameters )
{
    for(;;)
    {
    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Nucleo :  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_6);
    	vTaskDelay(1); // period Task3 200ms
    }
}


#endif
