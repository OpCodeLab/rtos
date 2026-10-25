/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : apk12.c
  * @brief          : APK12 - Signal processing: double buffer + CMSIS-DSP FFT
  ******************************************************************************
  *
  *  The PC (tools/apk12_gui.py) is a SENSOR: it sends the sum of 1, 2 or 3
  *  sine waves, ONE sample every 1 ms (fs = 1000 Hz).
  *  The STM32 computes the spectrum with CMSIS-DSP and sends it back.
  *
  *     UART RX ISR                         FftTask
  *     sample -> Buffer[Half][Index]
  *     half full ? ----- QueueHalf -----> FFT of the full half (arm_rfft_fast_f32)
  *     switch to the other half           |X[k]| -> frame -> PC
  *
  *  DOUBLE BUFFER: while the ISR fills one half, the FftTask reads the other.
  *  A new half is ready every FFT_N ms = 256 ms; the FFT takes < 1 ms.
  *  Resolution = fs / N = 3.9 Hz, max frequency = fs / 2 = 500 Hz.
  *
  *  Frames  PC -> STM32 : AA 50 LO HI               one int16 sample
  *          STM32 -> PC : AA 90 <128 x uint16>      amplitude of each bin
  *                        bin k = k * fs / N Hz, all values little endian
  *  CMSIS-DSP : Drivers/CMSIS/DSP (headers + libarm_cortexM4lf_math.a)
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"
#include "log.h"
#include "arm_math.h"

#ifdef APK12

#define FRAME_SOF      0xAAu
#define CMD_SAMPLE     0x50u
#define EVT_FFT        0x90u

#define FFT_N          256u                       /* 16..4096, power of 2 */
#define FFT_BINS       (FFT_N / 2u)

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

void FftTask( void * pvParameters );
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

void run_apk(void)
{
  xUartMutex = xSemaphoreCreateMutex();
  QueueHalf  = xQueueCreate(2, sizeof(uint8_t));

  arm_rfft_fast_init_f32(&Fft, FFT_N);

  xTaskCreate(FftTask, "FftTask", 512, NULL, 3, NULL);
  xTaskCreate(NoiseTask, "NoiseTask", 512, NULL, 4, NULL);

  LOG_INFO(COLOR_WHITE, " RTOS LAB APK12 (double buffer + CMSIS-DSP FFT %u) STARTING-----------", FFT_N);

  HAL_UART_Receive_IT(&huart2, &RxByte, 1);
  vTaskStartScheduler();

  while (1)
  {
  }
}

#if 1
uint32_t NoiseTaskDelay=60;
uint32_t LoadNoise=30;
void NoiseTask( void * pvParameters )
{
    while (1)
   {
     HAL_Delay(LoadNoise);
     vTaskDelay(NoiseTaskDelay);
   }

}
#endif

/* ========================================================= FftTask =======*/
void FftTask(void *p)
{
  uint8_t  half;
  uint32_t k, v;

  while (1)
  {
    xQueueReceive(QueueHalf, &half, portMAX_DELAY);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
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

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);       /* green LED: one FFT done */
  }
}


/* ====================================================== UART RX ISR =====*/
/* One sample received: store it in the current half, hand over a full half */
static void StoreSample(int16_t s, BaseType_t *xHigherPriority)
{
  Buffer[Half][Index++] = s;

  if (Index >= FFT_N)
  {
    xQueueSendFromISR(QueueHalf, &Half, xHigherPriority);
    Half ^= 1u;                                   /* ping <-> pong */
    Index = 0;
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);       /* New HalfFrameST */

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
