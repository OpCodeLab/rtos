/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : apk15.c
  * @brief          : APK15 - APK12 with TWO sensors: one double buffer and one
  *                   FFT task per sensor
  ******************************************************************************
  *
  *  The PC (tools/apk15_gui.py) simulates TWO sensors. Every 1 ms (fs = 1000 Hz)
  *  it sends one sample of each sensor, the sensor is given by the command:
  *
  *     AA 51 LO HI   sample of sensor 1
  *     AA 52 LO HI   sample of sensor 2
  *
  *  StoreSample() (UART RX ISR) dispatches each sample to the double buffer of
  *  ITS sensor. When a half is full, it wakes the FFT task of THAT sensor only:
  *
  *     UART RX ISR                                FFT tasks
  *     AA 51 -> Sensor1.Buffer ---- Sensor1.Queue --> FftSensor1 -> AA 91 <spectrum>
  *     AA 52 -> Sensor2.Buffer ---- Sensor2.Queue --> FftSensor2 -> AA 92 <spectrum>
  *
  *  ONE TASK FUNCTION, TWO TASKS: FftTask() is created twice, each instance
  *  receives a pointer to its own Sensor_t (buffers, queue, LED, frame). The
  *  code is shared, the data is not: this is why every buffer the task writes
  *  lives in Sensor_t and not in a static of the function. Only the FFT
  *  instance (read-only tables after init) is shared.
  *
  *  The two tasks share the UART for TX: xUartMutex keeps the frames whole.
  *
  *  Frames  STM32 -> PC : AA 91 <128 x uint16>   spectrum of sensor 1
  *                        AA 92 <128 x uint16>   spectrum of sensor 2
  *                        bin k = k * fs / N Hz, all values little endian
  *
  *  Bandwidth @115200: RX 2 x 4 bytes x 1000/s = 8000 B/s (70 %),
  *                     TX 2 x 258 bytes every 256 ms = 2 kB/s.
  *
  *  LEDs : PD12 green  FftSensor1 running     PD13 orange  sensor 1 half full
  *         PD15 blue   FftSensor2 running     PD14 red     sensor 2 half full
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"
#include "log.h"
#include "arm_math.h"

#ifdef APK15

#define FRAME_SOF      0xAAu
#define CMD_SAMPLE_S1  0x51u
#define CMD_SAMPLE_S2  0x52u
#define EVT_FFT        0x90u                      /* spectrum frame: AA <EVT_FFT + SensorId> */

#define FFT_N          256u                       /* 16..4096, power of 2 */
#define FFT_BINS       (FFT_N / 2u)

typedef enum { WAIT_SOF = 0, WAIT_CMD, WAIT_LO, WAIT_HI } RxState_t;

/* Everything that belongs to ONE sensor: ISR side + FFT task side */
typedef struct
{
  uint8_t       SensorId;                         /* 1 or 2 */
  uint16_t      LedFft;                           /* toggled by its FFT task */
  uint16_t      LedHalf;                          /* toggled when a half is full */

  int16_t       Buffer[2][FFT_N];                 /* the double buffer */
  uint8_t       Half;                             /* half written by the ISR */
  uint16_t      Index;
  QueueHandle_t Queue;                            /* ISR -> FFT task : index of the full half */

  float32_t     FftIn[FFT_N], FftOut[FFT_N], Mag[FFT_BINS];
  uint8_t       TxFrame[2u + 2u * FFT_BINS];
} Sensor_t;

extern UART_HandleTypeDef huart2;

static Sensor_t Sensor1 = { .SensorId = 1, .LedFft = GPIO_PIN_12, .LedHalf = GPIO_PIN_13 };
static Sensor_t Sensor2 = { .SensorId = 2, .LedFft = GPIO_PIN_15, .LedHalf = GPIO_PIN_14 };

static arm_rfft_fast_instance_f32 Fft;            /* shared: read-only after init */

static uint8_t   RxByte, RxLo, RxCmd;
static RxState_t RxState = WAIT_SOF;

void FftTask( void * pvParameters );


/* Spectrum -> AA <90 + SensorId> <128 x uint16>, amplitude A = 2 |X[k]| / N */
static void BuildFftFrame(Sensor_t *s)
{
  uint32_t k, v;

  s->TxFrame[0] = FRAME_SOF;
  s->TxFrame[1] = (uint8_t)(EVT_FFT + s->SensorId);   /* 0x91 sensor 1, 0x92 sensor 2 */
  for (k = 0; k < FFT_BINS; k++)
  {
    v = (uint32_t)(s->Mag[k] * 2.0f / FFT_N + 0.5f);
    if (v > 0xFFFFu)
      v = 0xFFFFu;
    s->TxFrame[2u + 2u * k]      = (uint8_t)(v & 0xFFu);
    s->TxFrame[2u + 2u * k + 1u] = (uint8_t)(v >> 8);
  }
}


void run_apk(void)
{
  LOG_INFO(COLOR_WHITE, " RTOS LAB APK15 two sensors / two FFT tasks STARTING-----------");

  xUartMutex = xSemaphoreCreateMutex();
  arm_rfft_fast_init_f32(&Fft, FFT_N);

  Sensor1.Queue = xQueueCreate(2, sizeof(uint8_t));
  Sensor2.Queue = xQueueCreate(2, sizeof(uint8_t));

  xTaskCreate(FftTask, "FftSensor1", 512, &Sensor1, 4, NULL);
  xTaskCreate(FftTask, "FftSensor2", 512, &Sensor2, 3, NULL);

  HAL_UART_Receive_IT(&huart2, &RxByte, 1);
  vTaskStartScheduler();

  while (1)
  {
  }
}


/* ========================================================= FftTask =======*/
/* Created once per sensor: p = its Sensor_t */
void FftTask(void *p)
{
  Sensor_t *s = (Sensor_t *)p;
  uint8_t   half;
  uint32_t  k;

  while (1)
  {
    xQueueReceive(s->Queue, &half, portMAX_DELAY);
    HAL_GPIO_TogglePin(GPIOD, s->LedFft);

    /* 1. int16 samples -> float */
    for (k = 0; k < FFT_N; k++)
      s->FftIn[k] = (float32_t)s->Buffer[half][k];

    /* 2. real FFT: FftOut = { Re0, Re(N/2), Re1, Im1, Re2, Im2, ... } */
    arm_rfft_fast_f32(&Fft, s->FftIn, s->FftOut, 0);

    /* 3. |X[k]| for k = 0 .. N/2-1 */
    arm_cmplx_mag_f32(s->FftOut, s->Mag, FFT_BINS);
    s->Mag[0] = fabsf(s->FftOut[0]) / 2.0f;      /* bin 0 = DC only (slot 1 holds N/2) */

    /* 4. amplitude in sample units: A = 2 |X[k]| / N, as uint16 */
    BuildFftFrame(s);

    xSemaphoreTake(xUartMutex, portMAX_DELAY);    /* the other FFT task may be sending */
    HAL_UART_Transmit(&huart2, s->TxFrame, sizeof(s->TxFrame), 100);
    xSemaphoreGive(xUartMutex);

    HAL_GPIO_TogglePin(GPIOD, s->LedFft);         /* one FFT done */
  }
}


/* ====================================================== UART RX ISR =====*/
/* One sample: the command tells the sensor -> store it in ITS current half,
   wake ITS FFT task */
static void StoreSample(uint8_t cmd, int16_t sample, BaseType_t *xHigherPriority)
{
  Sensor_t *s = (cmd == CMD_SAMPLE_S1) ? &Sensor1 : &Sensor2;

  s->Buffer[s->Half][s->Index++] = sample;

  if (s->Index >= FFT_N)
  {
    xQueueSendFromISR(s->Queue, &s->Half, xHigherPriority);
    s->Half ^= 1u;                                /* ping <-> pong */
    s->Index = 0;
    HAL_GPIO_TogglePin(GPIOD, s->LedHalf);
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
      RxCmd   = RxByte;
      RxState = ((RxCmd == CMD_SAMPLE_S1) || (RxCmd == CMD_SAMPLE_S2)) ? WAIT_LO : WAIT_SOF;
      break;

    case WAIT_LO:
      RxLo    = RxByte;
      RxState = WAIT_HI;
      break;

    case WAIT_HI:
      StoreSample(RxCmd, (int16_t)(uint16_t)(RxLo | (RxByte << 8)), &xHigherPriority);
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
