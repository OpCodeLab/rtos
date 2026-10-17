/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : apk7.c
  * @brief          : APK7 - Event group + software timer on top of the APK5 link
  *                   REFERENCE SOLUTION (trainer). Students start from apk7_student.c
  ******************************************************************************
  *
  *  Frame (3 bytes) : | SOF 0xAA | CMD | DATA |
  *
  *  PC -> STM32 (answered with CMD|0x80, DATA = status)
  *     0x01 PING        data echoed
  *     0x40 BUTTON_1    sets event bit EV_BUTTON_1
  *     0x41 BUTTON_2    sets event bit EV_BUTTON_2
  *
  *  STM32 -> PC (events)
  *     0x50 EVT_TIMER_STARTED   data = timer duration in seconds
  *     0x51 EVT_TIMER_EXPIRED   data = 0
  *
  *  Idea : Button 1 AND Button 2 received (any order)  ->  start a 5 s timer
  *         timer expires                               ->  tell the PC
  *
  *     LinkLayer task : frame -> xEventGroupSetBits(EV_BUTTON_x)
  *     App task       : wait EV_BUTTON_1 AND EV_BUTTON_2 -> start tApp
  *                      wait EV_TIMEOUT                   -> EVT_TIMER_EXPIRED
  *     tApp callback  : xEventGroupSetBits(EV_TIMEOUT)
  *
  *  LEDs : PD12 green  = button 1 received
  *         PD13 orange = button 2 received
  *         PD15 blue   = timer running
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"
#include "log.h"

#ifdef APK7

/* Protocol ------------------------------------------------------------------*/
#define FRAME_SOF            0xAAu
#define FRAME_RSP_FLAG       0x80u
#define FRAME_TIMEOUT_MS     50u

#define CMD_PING             0x01u
#define CMD_BUTTON_1         0x40u
#define CMD_BUTTON_2         0x41u
#define EVT_TIMER_STARTED    0x50u
#define EVT_TIMER_EXPIRED    0x51u

#define STATUS_OK            0x00u
#define STATUS_BAD_CMD       0xE1u

/* Application ---------------------------------------------------------------*/
#define TIMER_DURATION_MS    5000u

#define EV_BUTTON_1          (1u << 0)
#define EV_BUTTON_2          (1u << 1)
#define EV_TIMEOUT           (1u << 2)

#define LED_BUTTON_1         GPIO_PIN_12   /* green  */
#define LED_BUTTON_2         GPIO_PIN_13   /* orange */
#define LED_TIMER            GPIO_PIN_15   /* blue   */

typedef enum { WAIT_SOF = 0, WAIT_CMD, WAIT_DATA } RxState_t;

typedef struct
{
  uint8_t cmd;
  uint8_t data;
} Frame_t;

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

QueueHandle_t       QueueFrame;
EventGroupHandle_t  EgButtons;
TimerHandle_t       tApp;

static uint8_t    RxByte;
static RxState_t  RxState = WAIT_SOF;
static Frame_t    RxFrame;
static TickType_t LastByteTick;

void LinkLayer( void * pvParameters );
void AppTask( void * pvParameters );
static void AppTimerCallback( TimerHandle_t xTimer );
static void SendFrame(uint8_t cmd, uint8_t data);


void run_apk(void)
{
  xUartMutex = xSemaphoreCreateMutex();
  QueueFrame = xQueueCreate(8, sizeof(Frame_t));

  /* Event group: one bit per button + one bit for the timer */
  EgButtons = xEventGroupCreate();

  /* One-shot software timer, started by the AppTask */
  tApp = xTimerCreate("AppTimer", pdMS_TO_TICKS(TIMER_DURATION_MS), pdFALSE, NULL, AppTimerCallback);

  xTaskCreate(LinkLayer, "LinkLayer", 256, NULL, 3, NULL);
  xTaskCreate(AppTask,   "App",       256, NULL, 2, NULL);

  LOG_INFO(COLOR_WHITE, " RTOS LAB APK7 (event group + timer) STARTING-----------------");

  HAL_UART_Receive_IT(&huart2, &RxByte, 1);
  vTaskStartScheduler();

  while (1)
  {
  }
}


/* ======================================================= LinkLayer task ===*/
void LinkLayer(void *p)
{
  Frame_t frame;

  while (1)
  {
    xQueueReceive(QueueFrame, &frame, portMAX_DELAY);
    LOG_INFO(COLOR_CYAN, "RX frame CMD=0x%02X DATA=0x%02X", frame.cmd, frame.data);

    switch (frame.cmd)
    {
      case CMD_PING:
        SendFrame(frame.cmd | FRAME_RSP_FLAG, frame.data);
        break;

      case CMD_BUTTON_1:
        HAL_GPIO_WritePin(GPIOD, LED_BUTTON_1, GPIO_PIN_SET);
        xEventGroupSetBits(EgButtons, EV_BUTTON_1);
        SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_OK);
        break;

      case CMD_BUTTON_2:
        HAL_GPIO_WritePin(GPIOD, LED_BUTTON_2, GPIO_PIN_SET);
        xEventGroupSetBits(EgButtons, EV_BUTTON_2);
        SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_OK);
        break;

      default:
        SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_BAD_CMD);
        break;
    }
  }
}


/* ========================================================== App task ======*/
void AppTask(void *p)
{
  while (1)
  {
    /* 1. Wait until BOTH buttons were received, in any order (AND),
          and clear the two bits so the next round starts from zero */
    xEventGroupWaitBits(EgButtons, EV_BUTTON_1 | EV_BUTTON_2,
                        pdTRUE,          /* xClearOnExit     */
                        pdTRUE,          /* xWaitForAllBits  */
                        portMAX_DELAY);

    /* 2. Start the timer */
    HAL_GPIO_WritePin(GPIOD, LED_BUTTON_1 | LED_BUTTON_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, LED_TIMER, GPIO_PIN_SET);
    xTimerStart(tApp, 0);
    SendFrame(EVT_TIMER_STARTED, TIMER_DURATION_MS / 1000u);
    LOG_INFO(COLOR_GREEN, "Button 1 + Button 2 received: timer started (%u s)", TIMER_DURATION_MS / 1000u);

    /* 3. Wait for the timer callback */
    xEventGroupWaitBits(EgButtons, EV_TIMEOUT, pdTRUE, pdFALSE, portMAX_DELAY);

    HAL_GPIO_WritePin(GPIOD, LED_TIMER, GPIO_PIN_RESET);
    SendFrame(EVT_TIMER_EXPIRED, 0);
    LOG_INFO(COLOR_GREEN, "Timer expired");
  }
}


/* Runs in the timer service task: never block here */
static void AppTimerCallback(TimerHandle_t xTimer)
{
  (void)xTimer;
  xEventGroupSetBits(EgButtons, EV_TIMEOUT);
}


/* ====================================================== Link layer (APK5) =*/
static void SendFrame(uint8_t cmd, uint8_t data)
{
  uint8_t tx[3] = { FRAME_SOF, cmd, data };

  xSemaphoreTake(xUartMutex, portMAX_DELAY);
  HAL_UART_Transmit(&huart2, tx, sizeof(tx), 100);
  xSemaphoreGive(xUartMutex);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriority = pdFALSE;
  TickType_t now;

  if (huart->Instance != USART2)
    return;

  now = xTaskGetTickCountFromISR();
  if ((RxState != WAIT_SOF) && ((now - LastByteTick) > pdMS_TO_TICKS(FRAME_TIMEOUT_MS)))
    RxState = WAIT_SOF;
  LastByteTick = now;

  switch (RxState)
  {
    case WAIT_SOF:
      if (RxByte == FRAME_SOF)
        RxState = WAIT_CMD;
      break;

    case WAIT_CMD:
      RxFrame.cmd = RxByte;
      RxState     = WAIT_DATA;
      break;

    case WAIT_DATA:
      RxFrame.data = RxByte;
      xQueueSendFromISR(QueueFrame, &RxFrame, &xHigherPriority);
      RxState = WAIT_SOF;
      break;

    default:
      RxState = WAIT_SOF;
      break;
  }

  HAL_UART_Receive_IT(&huart2, &RxByte, 1);
  portYIELD_FROM_ISR(xHigherPriority);
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
    return;

  RxState = WAIT_SOF;
  HAL_UART_Receive_IT(&huart2, &RxByte, 1);
}

#endif
