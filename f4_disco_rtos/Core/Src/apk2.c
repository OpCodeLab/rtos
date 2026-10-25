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



#define GPIO_BT       GPIOA
#define GPIO_PIN_BT   GPIO_PIN_0



/**
  * @brief  The application entry point.
  * @retval int
  */
void run_apk(void)
{


   /*128 word Stack size / No Pamram / Priority=1, init hTask1->TCB1*/
  xTaskCreate(Task1,"Task1"  ,128, NULL,4,&hTask1);
  xTaskCreate(Task2,"Task2"  ,128,NULL,3,NULL);
  xTaskCreate(Task3,"Task3" ,128,NULL,3,NULL);

  Queue1 = xQueueCreate(6, sizeof (uint32_t));
  Sem1 = xSemaphoreCreateBinary();

  LOG_INFO(COLOR_WHITE," RTOS LAB STARTING-----------------");

  vTaskStartScheduler();

  while (1)
  {

  }
}

uint32_t debounceDelay=40;
uint32_t counter=0;
/*Asychrone  Task*/
void Task1(void *p) //P1
{
   // int Tab[2000];
  while(1)
  {
       xSemaphoreTake(Sem1 , portMAX_DELAY);
        vTaskDelay(debounceDelay);
       xSemaphoreTake(Sem1 , 0);
       counter++;

       HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);



  }
}
uint32_t ErrFull,ErrEmpty;
uint32_t ProducerFreq = 500;
uint32_t ConsumerFreq  = 500;

void Task2(void *p) // P3
{
  TickType_t  FirstTick,SecondTick;
  uint32_t PulseWidth;

  uint32_t DataTx=0;

  while(1)
  {
	//  LOG_INFO(COLOR_WHITE,"TAsk 2 is running....\n");
    HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);

    DataTx++;



    /*producer */
    if (xQueueSend(Queue1, (const void *) &DataTx, 0) == pdFALSE)
    {
       /*Quue is full*/
    	ErrFull++;
    }

    vTaskDelay(ProducerFreq);

  }
}//end task3




void Task3(void *p) // P4
{
  uint32_t PulseWidth;
  uint32_t DataRx=0;
 while(1)
 {
	//LOG_INFO(COLOR_WHITE,"TAsk 3 is running....\n");
   HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);

   if (xQueueReceive(Queue1, (void *) &DataRx, portMAX_DELAY) == pdFALSE)
       {
          /*Quue is full*/
       	ErrEmpty++;
       }

  // vTaskDelay(ConsumerFreq);



 }
}
uint32_t debouce=0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   BaseType_t xHigherPriority=pdFALSE;

   xSemaphoreGiveFromISR(Sem1, &xHigherPriority);
   /*if True force contex switch else wait for next tick*/
   portYIELD_FROM_ISR(xHigherPriority);
   debouce++;

}










#endif
