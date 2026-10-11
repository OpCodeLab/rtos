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
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PD */

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
#include "event_groups.h"

#define MOTOR_ERR1	         ( 1 << 0 )   // 0000000001
#define OVER_TEMP_ERR2	     ( 1 << 4 )   //   00010000

EventGroupHandle_t ErrEvent;

/**
  * @brief  The application entry point.
  * @retval int
  */
void run_apk(void)
{

  ErrEvent = xEventGroupCreate();

  /* Create a queue capable of containing 10 unsigned long values. */
  xQueue_UART = xQueueCreate( 10, sizeof( uint32_t ) );

  xUartMutex = xSemaphoreCreateMutex();

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
               //LOG_ERROR("<<PRODUCER>> : Send Queue Failed = %d", (int)errTX);
          	 xEventGroupSetBits(  ErrEvent,   MOTOR_ERR1 );
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

    	        if( xQueueReceive(xQueue_UART, (void *) &data_Rx, 0) != pdPASS )
    	        {
    	            Err_QueuEmpty++;
    	           // LOG_ERROR("<<CONSUMER>> : Queue Empty %d", (int)Err_QueuEmpty);
    	            xEventGroupSetBits(  ErrEvent,   OVER_TEMP_ERR2 );
    	        }
    	        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    	        counter++;

    	        vTaskDelay(Consumer_Period);
    }
}

void vTask3( void * pvParameters )
{
	 EventBits_t uxBits;

    for(;;)
    {

    	  uxBits = xEventGroupWaitBits( ErrEvent,   /* The event group being tested. */
    			                      MOTOR_ERR1 | OVER_TEMP_ERR2, /* The bits within the event group to wait for. */
    		                          pdTRUE,        /* BIT_0 & BIT_4 should be cleared before returning. */
    		                          pdFALSE,       /* Don't wait for both bits, either bit will do. */
    					              portMAX_DELAY );/* Wait a maximum of 100ms for either bit to be set. */

    	if (( uxBits & MOTOR_ERR1 ) != 0)
    	{
	      LOG_ERROR("<<ERROR>> : MOTOR1_ERROR ");
    	}
    	if (( uxBits & OVER_TEMP_ERR2 ) != 0)
    	{
    		LOG_ERROR("<<ERROR>> : TEMP_ERR ");
    	}


    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14); // Nucleo :  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_6);
    	vTaskDelay(1); // period Task3 200ms
    }
}


#endif
