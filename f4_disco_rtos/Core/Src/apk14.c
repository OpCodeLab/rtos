/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : apk14.c
  * @brief          : APK14 - APK12 + sensor AutoTest at start-up + SAFE mode
  ******************************************************************************
  *
  *  The PC (tools/apk14_gui.py) is the SENSOR. Before the application starts,
  *  the STM32 asks the sensor for its state (power-on self test):
  *
  *     STM32                               SENSOR (GUI)
  *     AA 60 cc   AUTOTEST_REQ  ------->   cc = challenge
  *                <-------  AUTOTEST_RSP   AA 61 ss ee
  *                                         ss = sensor state (0 = OK, else error bits)
  *                                         ee = cc ^ 0xFF   (proves the link works)
  *     AA 62 rr   STATUS        ------->   rr = 0 NORMAL, else SAFE + error bits
  *
  *     AutoTest OK   -> NORMAL : tasks created, scheduler started (= APK12)
  *     AutoTest FAIL -> SAFE   : no task, no FFT, red LED blinks, STATUS sent
  *                               every 1 s, waits for AA 63 RESET
  *
  *  Error bits (STATUS rr / AUTOTEST_RSP ss):
  *     0x01 POWER   sensor supply out of range        \
  *     0x02 STUCK   sensor output does not move        |  reported by the sensor
  *     0x04 RANGE   sensor output saturated            |
  *     0x08 CALIB   calibration data corrupted        /
  *     0x20 LOST    no sample for 500 ms (at run time: SupervisorTask)
  *     0x40 LINK    wrong echo in the answer           \  detected by the STM32
  *     0x80 NOANSW  no answer after 3 tries           /
  *
  *  WHY AutoTest() IS CALLED BEFORE ANY FreeRTOS FUNCTION:
  *     the first xQueueCreate / xSemaphoreCreate masks the interrupts with
  *     priority >= 5 (BASEPRI) until vTaskStartScheduler(). The HAL tick (TIM6,
  *     prio 15) would stop, HAL_GetTick() would freeze and every UART timeout
  *     would last forever. So the self test runs in bare metal, with polling.
  *
  *  Other frames:
  *     PC -> STM32 : AA 50 LO HI  one sample (NORMAL only)
  *                   AA 63 00     RESET: restart the board (and the AutoTest)
  *     STM32 -> PC : AA 90 <128 x uint16>   spectrum (NORMAL only)
  *
  *  LEDs : PD12 green  FftTask running (toggles)    PD13 orange  half buffer full
  *         PD14 red    SAFE (blinks)                PD15 blue    AutoTest running
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"
#include "log.h"
#include "arm_math.h"

#ifdef APK14

#define FRAME_SOF        0xAAu
#define CMD_SAMPLE       0x50u
#define CMD_AUTOTEST_REQ 0x60u
#define CMD_AUTOTEST_RSP 0x61u
#define EVT_STATUS       0x62u
#define CMD_RESET        0x63u
#define EVT_FFT          0x90u

#define FFT_N            256u                     /* 16..4096, power of 2 */
#define FFT_BINS         (FFT_N / 2u)

#define AUTOTEST_TRIES       3u
#define AUTOTEST_TIMEOUT_MS  500u                 /* per try */
#define STREAM_TIMEOUT_MS    500u                 /* SupervisorTask: max time without sample */
#define SUPERVISOR_PERIOD_MS 100u

#define ERR_POWER        0x01u
#define ERR_STUCK        0x02u
#define ERR_RANGE        0x04u
#define ERR_CALIB        0x08u
#define ERR_LOST         0x20u
#define ERR_LINK         0x40u
#define ERR_NOANSW       0x80u

#define LED_FFT          GPIO_PIN_12
#define LED_HALF         GPIO_PIN_13
#define LED_SAFE         GPIO_PIN_14
#define LED_AUTOTEST     GPIO_PIN_15

typedef enum { WAIT_SOF = 0, WAIT_CMD, WAIT_LO, WAIT_HI } RxState_t;

extern UART_HandleTypeDef huart2;
QueueHandle_t QueueHalf;                          /* ISR -> FftTask : index of the full half */

static int16_t   Buffer[2][FFT_N];                /* the double buffer */
static uint8_t   Half;                            /* half written by the ISR */
static uint16_t  Index;

static arm_rfft_fast_instance_f32 Fft;
static float32_t FftIn[FFT_N], FftOut[FFT_N], Mag[FFT_BINS];
static uint8_t   TxFrame[2u + 2u * FFT_BINS];

static uint8_t   RxByte, RxLo;
static RxState_t RxState = WAIT_SOF;

static volatile TickType_t SampleTick;            /* writer: ISR, reader: SupervisorTask */
static volatile uint8_t    SampleSeen;            /* supervision starts with the 1st sample */

volatile uint8_t AutoTestResult;                  /* 0 = OK (Live Expressions) */

void FftTask( void * pvParameters );
void SupervisorTask( void * pvParameters );
void NoiseTask( void * pvParameters );

static void BuildFftFrame(const float32_t *mag)
{
  uint32_t k, v;

  TxFrame[0] = FRAME_SOF;
  TxFrame[1] = EVT_FFT;

  for (k = 0; k < FFT_BINS; k++)
  {
    v = (uint32_t)(mag[k] * 2.0f / FFT_N + 0.5f);
    if (v > 0xFFFFu)
      v = 0xFFFFu;
    TxFrame[2u + 2u * k]      = (uint8_t)(v & 0xFFu);
    TxFrame[2u + 2u * k + 1u] = (uint8_t)(v >> 8);
  }
}
/* ======================================================= bare metal UART ==*/
/* Used before the scheduler (AutoTest) and in SAFE mode: polling, no interrupt */
static void SendFramePoll(uint8_t cmd, uint8_t data)
{
  uint8_t tx[3] = { FRAME_SOF, cmd, data };

  HAL_UART_Transmit(&huart2, tx, sizeof(tx), 50);
}


/* Wait for "AA <cmd> d0 [d1]" until deadline (HAL tick). Other bytes are skipped. */
static uint8_t ReceiveFramePoll(uint8_t cmd, uint8_t *data, uint8_t len, uint32_t deadline)
{
  uint8_t  b, state = 0, i = 0;
  uint32_t now;

  while ((now = HAL_GetTick()) < deadline)
  {
    if (HAL_UART_Receive(&huart2, &b, 1, deadline - now) != HAL_OK)
      continue;                                   /* timeout: the while() decides */
    switch (state)
    {
      case 0: state = (b == FRAME_SOF) ? 1u : 0u;     break;
      case 1: state = (b == cmd) ? 2u : 0u; i = 0;    break;
      default:
        data[i++] = b;
        if (i >= len)
          return 1;
        break;
    }
  }
  return 0;
}


/* ============================================================ AutoTest ===*/
/* Power-on self test of the sensor. Returns 0 if OK, else the error bits. */
uint8_t AutoTest(void)
{
  uint8_t  try, challenge, rsp[2];
  uint8_t  err = ERR_NOANSW;

  HAL_GPIO_WritePin(GPIOD, LED_AUTOTEST, GPIO_PIN_SET);
  __HAL_UART_CLEAR_OREFLAG(&huart2);              /* forget what arrived before */

  for (try = 1; try <= AUTOTEST_TRIES; try++)
  {
    challenge = (uint8_t)(0x5Au + try);           /* changes at each try */
    LOG_INFO(COLOR_YELLOW, "AutoTest: try %u/%u, challenge 0x%02X", try, AUTOTEST_TRIES, challenge);
    SendFramePoll(CMD_AUTOTEST_REQ, challenge);

    if (!ReceiveFramePoll(CMD_AUTOTEST_RSP, rsp, 2, HAL_GetTick() + AUTOTEST_TIMEOUT_MS))
    {
      err = ERR_NOANSW;                           /* sensor silent: try again */
      continue;
    }
    if (rsp[1] != (uint8_t)(challenge ^ 0xFFu))
    {
      err = ERR_LINK;                             /* answer corrupted: try again */
      continue;
    }
    err = rsp[0];                                 /* the sensor answered: its state is final */
    break;
  }

  HAL_GPIO_WritePin(GPIOD, LED_AUTOTEST, GPIO_PIN_RESET);
  AutoTestResult = err;
  return err;
}


/* ============================================================ SAFE mode ===*/
/* Bare metal, never returns. No task, no FFT: only the red LED and the RESET. */
static void SafeMode(uint8_t err)
{
  uint8_t  cmd;
  uint32_t lastStatus = 0;

  HAL_GPIO_WritePin(GPIOD, LED_FFT | LED_HALF | LED_AUTOTEST, GPIO_PIN_RESET);
  LOG_ERROR("SAFE MODE: error 0x%02X - send AA 63 00 to restart", err);

  while (1)
  {
    if (HAL_GetTick() - lastStatus >= 1000u)      /* repeat the STATUS for a late GUI */
    {
      lastStatus = HAL_GetTick();
      SendFramePoll(EVT_STATUS, err);
    }
    HAL_GPIO_TogglePin(GPIOD, LED_SAFE);          /* blink 4 Hz, the wait is the RX timeout */
    if (ReceiveFramePoll(CMD_RESET, &cmd, 1, HAL_GetTick() + 125u))
      NVIC_SystemReset();
  }
}


void run_apk(void)
{
  uint8_t ret;

  /* 1. Self test FIRST: bare metal, before any FreeRTOS call (see header) */
  LOG_INFO(COLOR_WHITE, " RTOS LAB APK14 AutoTest STARTING-----------");
  ret = AutoTest();

  SendFramePoll(EVT_STATUS, ret);
  if (ret != 0)
  {
    SafeMode(ret);                                /* Failure Mode: never returns */
  }
  LOG_INFO(COLOR_GREEN, "AutoTest OK: NORMAL mode, starting the application");

  /* 2. Sensor OK: same application as APK12 */
  xUartMutex = xSemaphoreCreateMutex();
  QueueHalf  = xQueueCreate(2, sizeof(uint8_t));
  arm_rfft_fast_init_f32(&Fft, FFT_N);

  xTaskCreate(FftTask, "FftTask", 512, NULL, 3, NULL);
  xTaskCreate(SupervisorTask, "Supervisor", 512, NULL, 5, NULL);

  HAL_UART_Receive_IT(&huart2, &RxByte, 1);
  vTaskStartScheduler();

  while (1)
  {
  }
}


/* ====================================================== SupervisorTask ===*/
/* Run-time check of the sensor: samples must keep coming once started */
void SupervisorTask(void *p)
{
  TickType_t lastWake = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SUPERVISOR_PERIOD_MS));

    if (SampleSeen && ((lastWake - SampleTick) > pdMS_TO_TICKS(STREAM_TIMEOUT_MS)))
    {
      LOG_ERROR("Supervisor: no sample for %u ms -> SAFE", STREAM_TIMEOUT_MS);
      xSemaphoreTake(xUartMutex, portMAX_DELAY);  /* FftTask is not in the middle of a frame */
      vTaskSuspendAll();                          /* stop the application ...            */
      HAL_UART_AbortReceive(&huart2);             /* ... and the interrupt reception     */
      SafeMode(ERR_LOST);                         /* never returns                       */
    }
  }
}


/* ========================================================= FftTask =======*/
void FftTask(void *p)
{
  uint8_t  half;
  uint32_t k, v;

  while (1)
  {
    xQueueReceive(QueueHalf, &half, portMAX_DELAY);
    HAL_GPIO_TogglePin(GPIOD, LED_FFT);
    /* 1. int16 samples -> float */
    for (k = 0; k < FFT_N; k++)
      FftIn[k] = (float32_t)Buffer[half][k];

    /* 2. real FFT: FftOut = { Re0, Re(N/2), Re1, Im1, Re2, Im2, ... } */
    arm_rfft_fast_f32(&Fft, FftIn, FftOut, 0);

    /* 3. |X[k]| for k = 0 .. N/2-1 */
    arm_cmplx_mag_f32(FftOut, Mag, FFT_BINS);
    Mag[0] = fabsf(FftOut[0]) / 2.0f;            /* bin 0 = DC only (slot 1 holds N/2) */

    /* 4. amplitude in sample units: A = 2 |X[k]| / N, as uint16 */
    BuildFftFrame(Mag);

    xSemaphoreTake(xUartMutex, portMAX_DELAY);
    HAL_UART_Transmit(&huart2, TxFrame, sizeof(TxFrame), 100);
    xSemaphoreGive(xUartMutex);

    HAL_GPIO_TogglePin(GPIOD, LED_FFT);           /* green LED: one FFT done */
  }
}


/* ====================================================== UART RX ISR =====*/
/* One sample received: store it in the current half, hand over a full half */
static void StoreSample(int16_t s, BaseType_t *xHigherPriority)
{
  Buffer[Half][Index++] = s;
  SampleTick = xTaskGetTickCountFromISR();
  SampleSeen = 1;

  if (Index >= FFT_N)
  {
    xQueueSendFromISR(QueueHalf, &Half, xHigherPriority);
    Half ^= 1u;                                   /* ping <-> pong */
    Index = 0;
    HAL_GPIO_TogglePin(GPIOD, LED_HALF);          /* New HalfFrameST */
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriority = pdFALSE;

  if (huart->Instance != USART2)
    return;

  switch (RxState)
  {
    case WAIT_SOF:
      if (RxByte == FRAME_SOF)
        RxState = WAIT_CMD;
      break;

    case WAIT_CMD:
      if (RxByte == CMD_RESET)                    /* GUI asks for a new AutoTest */
        NVIC_SystemReset();
      RxState = (RxByte == CMD_SAMPLE) ? WAIT_LO : WAIT_SOF;
      break;

    case WAIT_LO:
      RxLo    = RxByte;
      RxState = WAIT_HI;
      break;

    case WAIT_HI:
      StoreSample((int16_t)(uint16_t)(RxLo | (RxByte << 8)), &xHigherPriority);
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
