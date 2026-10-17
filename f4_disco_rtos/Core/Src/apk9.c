/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : apk9.c
  * @brief          : APK9 - Simple safety architecture: NORMAL / SAFE
  *                   (simplified version of APK8, same water heater, same GUI idea)
  ******************************************************************************
  *
  *  The PC (tools/apk9_gui.py) simulates the water and sends the temperature
  *  every 100 ms. The STM32 keeps the water at 40 C with a heater.
  *
  *  SAFETY GOAL: never heat when the water is too hot or its temperature unknown.
  *
  *  3 tasks, each with ONE job:
  *     LinkLayer   (prio 3) : receives frames, stores the last temperature
  *     ControlTask (prio 2) : every 100 ms computes the heater power it WANTS
  *     SafetyTask  (prio 4) : every 50 ms checks for errors, owns the MODE and
  *                            is the ONLY task that writes the outputs
  *
  *  2 modes:
  *     NORMAL : outputs = what the ControlTask requests, relay closed
  *     SAFE   : heater OFF, relay OPEN, wait for a RESET request
  *
  *  4 errors, any of them -> SAFE:
  *     ERR_SENSOR   sensor value 0xFF or > 120 C   (wire cut)
  *     ERR_TIMEOUT  no sample for 500 ms           (cable lost)
  *     ERR_OVERTEMP temperature > 90 C             (heater stuck ON)
  *     ERR_ALIVE    ControlTask stopped 300 ms     (software bug)
  *
  *  Frames  PC -> STM32 : 0x01 PING, 0x60 SENSOR_TEMP (no answer), 0x62 RESET_REQ,
  *                        0x63 INJECT_FAULT (1 = deadlock), 0x64 GET_MODE
  *          STM32 -> PC : 0x70 EVT_MODE (0 NORMAL, 1 SAFE), 0x71 EVT_ERROR (bits),
  *                        0x72 EVT_HEATER (power %), 0x73 EVT_RELAY (1 closed), 0x74 EVT_BOOT
  *  LEDs : PD12 green NORMAL, PD14 red SAFE, PD15 blue heater ON
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "queue.h"
#include "semphr.h"
#include "log.h"

#ifdef APK9

/* Protocol ------------------------------------------------------------------*/
#define FRAME_SOF            0xAAu
#define FRAME_RSP_FLAG       0x80u
#define FRAME_TIMEOUT_MS     50u

#define CMD_PING             0x01u
#define CMD_SENSOR_TEMP      0x60u
#define CMD_RESET_REQ        0x62u
#define CMD_INJECT_FAULT     0x63u
#define CMD_GET_MODE         0x64u

#define EVT_MODE             0x70u
#define EVT_ERROR            0x71u
#define EVT_HEATER           0x72u
#define EVT_RELAY            0x73u
#define EVT_BOOT             0x74u

#define STATUS_OK            0x00u
#define STATUS_BAD_CMD       0xE1u
#define STATUS_RESET_REFUSED 0xE6u

/* Application ---------------------------------------------------------------*/
#define SET_POINT_C          40u
#define HYSTERESIS_C         2u
#define CONTROL_PERIOD_MS    100u
#define SAFETY_PERIOD_MS     50u

#define SENSOR_INVALID       0xFFu
#define SENSOR_MAX_C         120u
#define OVERTEMP_C           90u      /* detection, hazard is 95 C */
#define SENSOR_TIMEOUT_MS    500u
#define ALIVE_TIMEOUT_MS     300u
#define RESET_MAX_TEMP_C     60u

#define ERR_SENSOR           (1u << 0)
#define ERR_TIMEOUT          (1u << 1)
#define ERR_OVERTEMP         (1u << 2)
#define ERR_ALIVE            (1u << 3)

#define LED_NORMAL           GPIO_PIN_12
#define LED_SAFE             GPIO_PIN_14
#define LED_HEATER           GPIO_PIN_15

typedef enum { WAIT_SOF = 0, WAIT_CMD, WAIT_DATA } RxState_t;
typedef enum { MODE_NORMAL = 0, MODE_SAFE } Mode_t;
typedef struct { uint8_t cmd; uint8_t data; } Frame_t;

/* Shared data: every variable has ONE writer -------------------------------*/
extern UART_HandleTypeDef huart2;
QueueHandle_t     QueueFrame;
SemaphoreHandle_t SemNeverGiven;                  /* used to simulate a deadlock */

static volatile uint8_t    SensorValue = SENSOR_INVALID;  /* writer: LinkLayer   */
static volatile TickType_t SensorTick;                    /* writer: LinkLayer   */
static volatile uint8_t    SensorSeen;                    /* writer: LinkLayer   */
static volatile uint8_t    ResetRequest;                  /* writer: LinkLayer (set), SafetyTask (clear) */
static volatile uint8_t    InjectDeadlock;                /* writer: LinkLayer   */
static volatile uint8_t    PowerRequest;                  /* writer: ControlTask */
static volatile uint32_t   ControlAlive;                  /* writer: ControlTask */
static volatile Mode_t     Mode = MODE_NORMAL;            /* writer: SafetyTask  */

static uint8_t    RxByte;
static RxState_t  RxState = WAIT_SOF;
static Frame_t    RxFrame;
static TickType_t LastByteTick;

void LinkLayer( void * pvParameters );
void ControlTask( void * pvParameters );
void SafetyTask( void * pvParameters );
static void SendFrame(uint8_t cmd, uint8_t data);


void run_apk(void)
{
  xUartMutex    = xSemaphoreCreateMutex();
  QueueFrame    = xQueueCreate(8, sizeof(Frame_t));
  SemNeverGiven = xSemaphoreCreateBinary();

  xTaskCreate(SafetyTask,  "Safety",    256, NULL, 4, NULL);
  xTaskCreate(LinkLayer,   "LinkLayer", 256, NULL, 3, NULL);
  xTaskCreate(ControlTask, "Control",   256, NULL, 2, NULL);

  LOG_INFO(COLOR_WHITE, " RTOS LAB APK9 (simple safety: NORMAL / SAFE) STARTING-----------");

  HAL_UART_Receive_IT(&huart2, &RxByte, 1);
  vTaskStartScheduler();

  while (1)
  {
  }
}


/* ====================================================== ControlTask ======*/
/* The FUNCTION: computes the heater power it wants. It never touches the outputs. */
void ControlTask(void *p)
{
  TickType_t lastWake = xTaskGetTickCount();
  uint8_t    power = 0;
  uint8_t    t;

  while (1)
  {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    ControlAlive++;                                   /* "I am still running" */

    if (InjectDeadlock)                               /* simulated software bug */
    {
      LOG_ERROR("ControlTask: deadlock (injected fault)");
      xSemaphoreTake(SemNeverGiven, portMAX_DELAY);  /* never returns */
    }

    t = SensorValue;
    if (t > SENSOR_MAX_C)
    {
    	power = 0;     /* unknown T: do not heat */
    }
    else if (t + HYSTERESIS_C <= SET_POINT_C)
    {
      power = 100;
    }
    else if (t >= SET_POINT_C)
    {
      power = 0;     /* else: keep last power  */
    }

    PowerRequest = power;
  }
}


/* ======================================================= SafetyTask ======*/
/* Look for the 4 errors. Returns a bit mask, 0 = everything is fine. */
static uint8_t CheckErrors(TickType_t now, uint32_t *lastAlive, TickType_t *aliveTick)
{
  uint8_t err = 0;

  if (SensorSeen)                                     /* supervision starts with the 1st sample */
  {
    if (SensorValue > SENSOR_MAX_C)
     {
      err |= ERR_SENSOR;
      }
    else if (SensorValue > OVERTEMP_C)
    {
    	err |= ERR_OVERTEMP;
    }
    if ((now - SensorTick) > pdMS_TO_TICKS(SENSOR_TIMEOUT_MS))
      {
     err |= ERR_TIMEOUT;
    	}
  }

  if (ControlAlive != *lastAlive)                     /* is the ControlTask still alive? */
  {
    *lastAlive = ControlAlive;
    *aliveTick = now;
  }
  else if ((now - *aliveTick) > pdMS_TO_TICKS(ALIVE_TIMEOUT_MS))
  {
    err |= ERR_ALIVE;
  }
  return err;
}


/* The ONLY place where the outputs are written */
static void WriteOutputs(uint8_t power, uint8_t relayClosed)
{
  static uint8_t lastPower = 0xFF, lastRelay = 0xFF;

  HAL_GPIO_WritePin(GPIOD, LED_HEATER, power ? GPIO_PIN_SET : GPIO_PIN_RESET);
  if (power != lastPower)       { lastPower = power;       SendFrame(EVT_HEATER, power); }
  if (relayClosed != lastRelay) { lastRelay = relayClosed; SendFrame(EVT_RELAY, relayClosed); }
}


void SafetyTask(void *p)
{
  TickType_t lastWake = xTaskGetTickCount();
  TickType_t aliveTick = lastWake;
  uint32_t   lastAlive = ControlAlive;
  uint8_t    err;

  SendFrame(EVT_BOOT, 0);
  SendFrame(EVT_MODE, MODE_NORMAL);
  HAL_GPIO_WritePin(GPIOD, LED_NORMAL, GPIO_PIN_SET);



  while (1)
  {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    err = CheckErrors(lastWake, &lastAlive, &aliveTick);

     //HAL_IWDG_Refresh(&hiwdg);

    switch (Mode)
    {
      case MODE_NORMAL:
        if (err == 0)
        {
          WriteOutputs(PowerRequest, 1);              /* follow the ControlTask */
          break;
        }
        /* any error: go to SAFE */
        Mode = MODE_SAFE;
        WriteOutputs(0, 0);                           /* heater OFF, relay OPEN, first */
        HAL_GPIO_WritePin(GPIOD, LED_NORMAL, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LED_SAFE, GPIO_PIN_SET);
        SendFrame(EVT_MODE, MODE_SAFE);
        SendFrame(EVT_ERROR, err);
        LOG_ERROR("SAFE: errors 0x%02X", err);
        break;

      case MODE_SAFE:
        WriteOutputs(0, 0);                           /* stay OFF whatever happens */
        if (ResetRequest)
        {
          ResetRequest = 0;
          if (SensorSeen && ((lastWake - SensorTick) < pdMS_TO_TICKS(SENSOR_TIMEOUT_MS)) &&
              (SensorValue < RESET_MAX_TEMP_C))
          {
            SendFrame(CMD_RESET_REQ | FRAME_RSP_FLAG, STATUS_OK);
            LOG_INFO(COLOR_GREEN, "Reset accepted: restart");
            vTaskDelay(pdMS_TO_TICKS(20));            /* let the UART finish */
            NVIC_SystemReset();                       /* clean restart = NORMAL */
          }
          LOG_ERROR("Reset refused: sensor not valid or T >= %u C", RESET_MAX_TEMP_C);
          SendFrame(CMD_RESET_REQ | FRAME_RSP_FLAG, STATUS_RESET_REFUSED);
        }
        break;
    }
  }
}


/* ======================================================= LinkLayer task ===*/
void LinkLayer(void *p)
{
  Frame_t frame;

  while (1)
  {
    xQueueReceive(QueueFrame, &frame, portMAX_DELAY);

    switch (frame.cmd)
    {
      case CMD_SENSOR_TEMP:                           /* periodic data: no answer */
        SensorValue = frame.data;
        SensorTick  = xTaskGetTickCount();
        SensorSeen  = 1;
        break;

      case CMD_PING:
        SendFrame(frame.cmd | FRAME_RSP_FLAG, frame.data);
        break;

      case CMD_RESET_REQ:                             /* the SafetyTask decides and answers */
        if (Mode == MODE_SAFE)
          ResetRequest = 1;
        else
          SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_RESET_REFUSED);
        break;

      case CMD_INJECT_FAULT:
        InjectDeadlock = (frame.data == 1);
        SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_OK);
        break;

      case CMD_GET_MODE:
        SendFrame(frame.cmd | FRAME_RSP_FLAG, (uint8_t)Mode);
        break;

      default:
        SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_BAD_CMD);
        break;
    }
  }
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
