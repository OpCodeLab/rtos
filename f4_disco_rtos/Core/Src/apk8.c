/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : apk8.c
  * @brief          : APK8 - SW architecture for functional safety
  *                   Water heater controller with NORMAL / DEGRADED / SAFE / RESET
  ******************************************************************************
  *
  *  Real case : keep water at a set point with a heater. The temperature sensor
  *  is simulated by the PC (tools/apk8_gui.py) through the APK5 link layer.
  *
  *  Fault  -> Error  -> Failure   (what the architecture must stop)
  *  ---------------------------------------------------------------------------
  *  Fault (root cause, injected)   Error (detected here)       Reaction
  *  sensor wire cut   (0xFF)       ERR_RANGE    range check      DEGRADED
  *  EMI spike         (+40 C)      ERR_GRADIENT plausibility     DEGRADED
  *  cable / PC lost                ERR_TIMEOUT  no sample 500ms  SAFE
  *  heater relay stuck ON          ERR_OVERTEMP T > 90 C         SAFE
  *  SW bug: deadlock               ERR_ALIVE    alive counter    SAFE
  *  errors persist in DEGRADED     ERR_PERSIST  > 2 s            SAFE
  *  Failure avoided: "heater ON while the water is too hot or unknown"
  *
  *  Modes (SafetyManager owns the mode)
  *     NORMAL   : full control
  *     DEGRADED : last valid value kept, heater power limited to 50 %,
  *                back to NORMAL after 10 valid samples in a row
  *     SAFE     : heater 0 %, safety relay OPEN (second, independent path),
  *                outputs frozen, wait for RESET_REQ (external intervention)
  *     RESET    : RESET_REQ accepted only in SAFE with a fresh valid sample
  *                below 60 C -> NVIC_SystemReset()
  *
  *  Tasks
  *     SafetyManager (prio 4, 50 ms)  : alive monitor, error evaluation, modes
  *     LinkLayer     (prio 3)         : frames -> sensor mailbox / commands
  *     ControlTask   (prio 2, 100 ms) : error detection + heater control
  *
  *  Frames  PC -> STM32 : 0x01 PING, 0x60 SENSOR_TEMP (no answer), 0x61 SET_POINT,
  *                        0x62 RESET_REQ, 0x63 INJECT_FAULT (1 = deadlock), 0x64 GET_MODE
  *          STM32 -> PC : 0x70 EVT_MODE, 0x71 EVT_ERROR, 0x72 EVT_HEATER (power %),
  *                        0x73 EVT_RELAY (1 closed), 0x74 EVT_BOOT
  *  LEDs : PD12 green NORMAL, PD13 orange DEGRADED, PD14 red SAFE, PD15 blue heater on
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdlib.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "log.h"

#ifdef APK8

/* Protocol ------------------------------------------------------------------*/
#define FRAME_SOF            0xAAu
#define FRAME_RSP_FLAG       0x80u
#define FRAME_TIMEOUT_MS     50u

#define CMD_PING             0x01u
#define CMD_SENSOR_TEMP      0x60u
#define CMD_SET_POINT        0x61u
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
#define STATUS_BAD_DATA      0xE2u
#define STATUS_RESET_REFUSED 0xE6u

/* Safety parameters -----------------------------------------------------------*/
#define CONTROL_PERIOD_MS    100u
#define SAFETY_PERIOD_MS     50u
#define SENSOR_INVALID       0xFFu
#define SENSOR_MAX_C         120u
#define MAX_GRADIENT_C       15u      /* per sample (100 ms)               */
#define OVERTEMP_C           90u
#define SENSOR_TIMEOUT_MS    500u
#define ALIVE_TIMEOUT_MS     300u
#define DEGRADED_MAX_MS      2000u
#define RECOVERY_SAMPLES     10u
#define RESET_MAX_TEMP_C     60u
#define DEGRADED_POWER       50u
#define HYSTERESIS_C         2u

/* Error bits (EgErrors) - set by ControlTask, read and cleared by SafetyManager */
#define ERR_RANGE            (1u << 0)
#define ERR_GRADIENT         (1u << 1)
#define ERR_TIMEOUT          (1u << 2)
#define ERR_OVERTEMP         (1u << 3)
#define ERR_ALIVE            (1u << 4)
#define ERR_PERSIST          (1u << 5)
#define EV_SAMPLE_OK         (1u << 6)
#define EV_RESET_REQ         (1u << 7)
#define ERR_TRANSIENT        (ERR_RANGE | ERR_GRADIENT)
#define ERR_CRITICAL         (ERR_TIMEOUT | ERR_OVERTEMP | ERR_ALIVE)

#define LED_NORMAL           GPIO_PIN_12
#define LED_DEGRADED         GPIO_PIN_13
#define LED_SAFE             GPIO_PIN_14
#define LED_HEATER           GPIO_PIN_15

/* Types ---------------------------------------------------------------------*/
typedef enum { WAIT_SOF = 0, WAIT_CMD, WAIT_DATA } RxState_t;
typedef enum { MODE_NORMAL = 0, MODE_DEGRADED, MODE_SAFE, MODE_RESET } Mode_t;

typedef struct { uint8_t cmd; uint8_t data; } Frame_t;
typedef struct { uint8_t value; TickType_t tick; } Sample_t;

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

QueueHandle_t       QueueFrame;
QueueHandle_t       MailSensor;          /* length 1, xQueueOverwrite = latest sample */
EventGroupHandle_t  EgErrors;
SemaphoreHandle_t   SemNeverGiven;       /* used to simulate a deadlock */

static volatile Mode_t   Mode = MODE_NORMAL;      /* written by SafetyManager only */
static volatile uint32_t ControlAlive;            /* written by ControlTask only   */
static volatile uint8_t  SetPoint = 40;
static volatile uint8_t  InjectDeadlock;
static volatile uint8_t  LastValidTemp;
static volatile uint8_t  HasValidSample;
static volatile uint8_t  RawValue = SENSOR_INVALID;  /* written by LinkLayer only: */
static volatile TickType_t RawTick;                  /* independent of ControlTask */
static volatile uint8_t  HeaterPower;             /* actual actuator value          */
static uint8_t           ReportedPower = 0xFF;    /* last EVT_HEATER sent (Safety)  */
static uint8_t           RelayClosed = 0xFF;

static uint8_t    RxByte;
static RxState_t  RxState = WAIT_SOF;
static Frame_t    RxFrame;
static TickType_t LastByteTick;

static const char *ModeName[] = { "NORMAL", "DEGRADED", "SAFE", "RESET" };

void LinkLayer( void * pvParameters );
void ControlTask( void * pvParameters );
void SafetyManager( void * pvParameters );
static void SendFrame(uint8_t cmd, uint8_t data);
static void Actuator_SetHeater(uint8_t power);
static void Actuator_SetRelay(uint8_t closed);
static void EnterMode(Mode_t m, uint8_t errBits);

void SafetyManager( void * pvParameters )
{
	while(1)
	{

	}

}


void run_apk(void)
{
  xUartMutex    = xSemaphoreCreateMutex();
  QueueFrame    = xQueueCreate(8, sizeof(Frame_t));
  MailSensor    = xQueueCreate(1, sizeof(Sample_t));
  EgErrors      = xEventGroupCreate();
  SemNeverGiven = xSemaphoreCreateBinary();

  xTaskCreate(SafetyManager, "Safety",    256, NULL, 4, NULL);
  xTaskCreate(LinkLayer,     "LinkLayer", 256, NULL, 3, NULL);
  xTaskCreate(ControlTask,   "Control",   256, NULL, 2, NULL);

  LOG_INFO(COLOR_WHITE, " RTOS LAB APK8 (functional safety) STARTING-----------------");

  HAL_UART_Receive_IT(&huart2, &RxByte, 1);
  vTaskStartScheduler();

  while (1)
  {
  }
}


/* ====================================================== Actuators ========*/
/* Single access point to the heater, refuses power in SAFE (defence in depth).
   Check + write are atomic: the SafetyManager (higher priority) cannot switch
   to SAFE between the mode check and the output write. No UART here: the
   SafetyManager is the only task that reports the heater state. */
static void Actuator_SetHeater(uint8_t power)
{
  taskENTER_CRITICAL();
  if (Mode == MODE_SAFE || Mode == MODE_RESET)
    power = 0;
  HeaterPower = power;
  HAL_GPIO_WritePin(GPIOD, LED_HEATER, power ? GPIO_PIN_SET : GPIO_PIN_RESET);
  taskEXIT_CRITICAL();
}

/* Safety relay = independent second path that cuts the heater supply */
static void Actuator_SetRelay(uint8_t closed)
{
  if (closed == RelayClosed)
    return;
  RelayClosed = closed;
  SendFrame(EVT_RELAY, closed);
}


/* ===================================================== Control task ======*/
void ControlTask(void *p)
{
  TickType_t lastWake = xTaskGetTickCount();
  TickType_t lastRxTick = 0;
  uint8_t    hasRx = 0;
  uint8_t    power = 0;
  Sample_t   s;

  while (1)
  {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    ControlAlive++;                                   /* heartbeat for SafetyManager */

    if (InjectDeadlock)                               /* simulated software bug */
    {
      LOG_ERROR("ControlTask: deadlock (injected fault)");
      xSemaphoreTake(SemNeverGiven, portMAX_DELAY);  /* never returns */
    }

    /* ---------- error detection on the sensor input ---------- */
    if (xQueueReceive(MailSensor, &s, 0) == pdPASS)
    {
      hasRx = 1;
      lastRxTick = s.tick;

      if ((s.value == SENSOR_INVALID) || (s.value > SENSOR_MAX_C))
      {
        xEventGroupSetBits(EgErrors, ERR_RANGE);
      }
      else if (HasValidSample && (abs((int)s.value - (int)LastValidTemp) > (int)MAX_GRADIENT_C))
      {
        xEventGroupSetBits(EgErrors, ERR_GRADIENT);       /* sample rejected */
      }
      else
      {
        LastValidTemp  = s.value;
        HasValidSample = 1;
        xEventGroupSetBits(EgErrors, EV_SAMPLE_OK);
        if (s.value > OVERTEMP_C)
          xEventGroupSetBits(EgErrors, ERR_OVERTEMP);
      }
    }

    /* timeout supervision starts with the first received sample */
    if (hasRx && ((xTaskGetTickCount() - lastRxTick) > pdMS_TO_TICKS(SENSOR_TIMEOUT_MS)))
      xEventGroupSetBits(EgErrors, ERR_TIMEOUT);

    /* ---------- function: on/off control with hysteresis ---------- */
    if (Mode == MODE_SAFE || Mode == MODE_RESET || !HasValidSample)
    {
      power = 0;                                      /* outputs frozen at 0 */
    }
    else
    {
      if (LastValidTemp + HYSTERESIS_C <= SetPoint) power = 100;
      else if (LastValidTemp >= SetPoint)          power = 0;
      if ((Mode == MODE_DEGRADED) && (power > DEGRADED_POWER))
        power = DEGRADED_POWER;                       /* limited functionality */
    }
    Actuator_SetHeater(power);
  }
}


/* =================================================== Safety manager ======*/
void SafetyManager(void *p)
{
  TickType_t lastWake = xTaskGetTickCount();
  TickType_t lastAliveTick = lastWake;
  TickType_t degradedSince = 0;
  uint32_t   lastAlive = ControlAlive;
  uint32_t   okCount = 0;
  uint8_t    reported = 0;          /* transient errors already reported in DEGRADED */
  EventBits_t bits;

  SendFrame(EVT_BOOT, 0);
  EnterMode(MODE_NORMAL, 0);

  while (1)
  {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAFETY_PERIOD_MS));



    /* collect and clear what happened since the last cycle */
    bits = xEventGroupClearBits(EgErrors, 0xFFu);

    /* alive monitoring of the ControlTask (temporal supervision) */
    if (ControlAlive != lastAlive)
    {
      lastAlive = ControlAlive;
      lastAliveTick = lastWake;
    }
    else if ((lastWake - lastAliveTick) > pdMS_TO_TICKS(ALIVE_TIMEOUT_MS))
    {
      bits |= ERR_ALIVE;
    }

    switch (Mode)
    {
      case MODE_NORMAL:
        if (bits & ERR_CRITICAL)
          EnterMode(MODE_SAFE, bits & ERR_CRITICAL);
        else if (bits & ERR_TRANSIENT)
        {
          degradedSince = lastWake;
          okCount = 0;
          reported = bits & ERR_TRANSIENT;
          EnterMode(MODE_DEGRADED, reported);
        }
        break;

      case MODE_DEGRADED:
        if (bits & ERR_CRITICAL)
          EnterMode(MODE_SAFE, bits & ERR_CRITICAL);
        else if ((lastWake - degradedSince) > pdMS_TO_TICKS(DEGRADED_MAX_MS))
          EnterMode(MODE_SAFE, ERR_PERSIST);
        else if (bits & ERR_TRANSIENT)
        {
          okCount = 0;
          if ((bits & ERR_TRANSIENT) & ~reported)        /* report a new error type once */
          {
            reported |= bits & ERR_TRANSIENT;
            SendFrame(EVT_ERROR, (uint8_t)(bits & ERR_TRANSIENT));
          }
        }
        else if ((bits & EV_SAMPLE_OK) && (++okCount >= RECOVERY_SAMPLES))
          EnterMode(MODE_NORMAL, 0);
        break;

      case MODE_SAFE:
        if (bits & EV_RESET_REQ)
        {
          /* Recovery condition uses the raw sensor path, NOT the ControlTask:
             if the ControlTask is the failed element, reset must still be possible */
          uint8_t fresh = (xTaskGetTickCount() - RawTick) < pdMS_TO_TICKS(SENSOR_TIMEOUT_MS);
          if (fresh && (RawValue <= SENSOR_MAX_C) && (RawValue < RESET_MAX_TEMP_C))
          {
            SendFrame(CMD_RESET_REQ | FRAME_RSP_FLAG, STATUS_OK);
            EnterMode(MODE_RESET, 0);
            vTaskDelay(pdMS_TO_TICKS(20));            /* let the UART drain */
            NVIC_SystemReset();
          }
          LOG_ERROR("Reset refused: sensor not valid/fresh or T >= %u C", RESET_MAX_TEMP_C);
          SendFrame(CMD_RESET_REQ | FRAME_RSP_FLAG, STATUS_RESET_REFUSED);
        }
        break;

      default:
        break;
    }

    /* report the actuator state (single reporter -> always consistent) */
    if (HeaterPower != ReportedPower)
    {
      ReportedPower = HeaterPower;
      SendFrame(EVT_HEATER, ReportedPower);
    }
  }
}


/* Mode change: outputs first, then report */
static void EnterMode(Mode_t m, uint8_t errBits)
{
  Mode = m;

  if (m == MODE_SAFE || m == MODE_RESET)
  {
    Actuator_SetHeater(0);          /* stop actuator          */
    Actuator_SetRelay(0);           /* independent cut-off    */
  }
  else
  {
    Actuator_SetRelay(1);
  }

  HAL_GPIO_WritePin(GPIOD, LED_NORMAL | LED_DEGRADED | LED_SAFE, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, (m == MODE_NORMAL) ? LED_NORMAL : (m == MODE_DEGRADED) ? LED_DEGRADED : LED_SAFE,
                    GPIO_PIN_SET);

  SendFrame(EVT_MODE, (uint8_t)m);
  if (errBits)
    SendFrame(EVT_ERROR, errBits);
  LOG_INFO((m == MODE_NORMAL) ? COLOR_GREEN : (m == MODE_DEGRADED) ? COLOR_YELLOW : COLOR_RED,
           "MODE %s (errors 0x%02X)", ModeName[m], errBits);
}


/* ======================================================= LinkLayer task ===*/
void LinkLayer(void *p)
{
  Frame_t  frame;
  Sample_t s;

  while (1)
  {
    xQueueReceive(QueueFrame, &frame, portMAX_DELAY);

    switch (frame.cmd)
    {
      case CMD_SENSOR_TEMP:                           /* periodic data: no answer */
        s.value = frame.data;
        s.tick  = xTaskGetTickCount();
        xQueueOverwrite(MailSensor, &s);
        RawValue = s.value;          /* for the reset check (ControlTask may be dead) */
        RawTick  = s.tick;
        break;

      case CMD_PING:
        SendFrame(frame.cmd | FRAME_RSP_FLAG, frame.data);
        break;

      case CMD_SET_POINT:
        if ((frame.data < 20) || (frame.data > 80))
        {
          SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_BAD_DATA);
          break;
        }
        SetPoint = frame.data;
        SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_OK);
        break;

      case CMD_RESET_REQ:                             /* SafetyManager decides */
        if (Mode != MODE_SAFE)
          SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_RESET_REFUSED);
        else
          xEventGroupSetBits(EgErrors, EV_RESET_REQ);
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
