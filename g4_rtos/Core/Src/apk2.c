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

#ifdef APK2
/* Private variables ---------------------------------------------------------*/




/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Task1( void * pvParameters );
void Task2( void * pvParameters );
void Task3( void * pvParameters );
/* USER CODE END PFP */
SemaphoreHandle_t   Sem1;
TaskHandle_t        hTask1;
QueueHandle_t       Queue1;



#define GPIO_BT       GPIOC
#define GPIO_PIN_BT   GPIO_PIN_13



/**
  * @brief  The application entry point.
  * @retval int
  */
void run_apk(void)
{


  /*CreateSemaphore */
  Sem1= xSemaphoreCreateBinary();
  /* Create a queue capable of containing 10 unsigned long values. */
  Queue1 = xQueueCreate( 10, sizeof( uint32_t ) );

  /*128 word Stack size / No Pamram / Priority=1, init hTask1->TCB1*/
  xTaskCreate(Task1,"Periodic"  ,128, NULL,2,&hTask1);
  xTaskCreate(Task2,"APeriodic"  ,128,NULL,3,NULL);
  xTaskCreate(Task3,"Processing" ,128,NULL,3,NULL);
  LOG_INFO(COLOR_WHITE," RTOS LAB STARTING-----------------");

  vTaskStartScheduler();

  while (1)
  {

  }
}



/*Perdidic Task*/
void Task1(void *p) //P1
{

  while(1)
  {
	  LOG_INFO(COLOR_WHITE,"TAsk 1 is running....\n");
    HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
    vTaskDelay(1000);
  }
}

void Task2(void *p) // P3
{
  TickType_t  FirstTick,SecondTick;
  uint32_t PulseWidth;
  while(1)
  {
   xSemaphoreTake(Sem1,portMAX_DELAY);
   /*Take the FirstTick*/
   FirstTick = xTaskGetTickCount();
   /*While button still forced ...*/
   while( HAL_GPIO_ReadPin(GPIO_BT,GPIO_PIN_BT)==GPIO_PIN_SET );
   SecondTick= xTaskGetTickCount();
   PulseWidth=SecondTick-FirstTick;
   xQueueSend(Queue1, (void*)&PulseWidth,100);
   //taskYIELD();
  }
}//end task3




void Task3(void *p) // P4
{
  uint32_t PulseWidth;

 while(1)
 {
    // Blocke Task until Queue receive a new data
    xQueueReceive(Queue1, &PulseWidth, portMAX_DELAY );
    if( (PulseWidth<=400) && (PulseWidth>100))
    {
    	LOG_INFO(COLOR_YELLOW,"Short Button detected ...\n");
    }
    else if (PulseWidth>400)
    {
    	LOG_INFO(COLOR_YELLOW,"Long Button detected ...\n");
    }
 }
}


void BSP_PB_Callback(Button_TypeDef Button)
{
	   BaseType_t xHigherPriority=pdFALSE;

	   xSemaphoreGiveFromISR(Sem1, &xHigherPriority);
	   /*if True force contex switch else wait for next tick*/
	   portYIELD_FROM_ISR(xHigherPriority);
}


#endif
