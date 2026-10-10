/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : apk5.c
  * @brief          : APK5 - Simple UART protocol between STM32 and a Python host
  ******************************************************************************
  *
  *  Frame format (3 bytes) :   | SOF | CMD | DATA |
  *                              0xAA
  *
  *  Response (STM32 -> PC)  :   | SOF | CMD | STATUS/DATA |   (CMD with bit7 set)
  *
  *  Architecture :
  *
  *   USART2 RX IT (1 byte)          QueueFrame            LinkLayer task
  *  +-----------------------+     +-----------+      +----------------------+
  *  | HAL_UART_RxCpltCallback| --> | Frame_t   | ---> | decode CMD           |
  *  |  WAIT_SOF             |     | Frame_t   |      | execute handler      |
  *  |  WAIT_CMD             |     | ...       |      | send response frame  |
  *  |  WAIT_DATA            |     +-----------+      +----------------------+
  *  +-----------------------+
  *
  *  NOTE: USART2 is also used by printf (LOG_INFO). The log text is pure ASCII
  *  (< 0x80) so SOF = 0xAA never appears inside it: the host resyncs on SOF.
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

#ifdef APK5

/* Protocol definitions ------------------------------------------------------*/
#define FRAME_SOF            0xAAu
#define FRAME_RSP_FLAG       0x80u   /* response CMD = request CMD | 0x80   */
#define FRAME_TIMEOUT_MS     50u     /* max gap between 2 bytes of a frame  */
#define QUEUE_FRAME_LEN      8u

/* Commands (PC -> STM32) */
#define CMD_PING             0x01u   /* DATA echoed back                    */
#define CMD_LED_SET          0x02u   /* DATA bit0..3 -> LED PD12..PD15      */
#define CMD_LED_TOGGLE       0x03u   /* DATA = LED index 0..3               */
#define CMD_GET_BUTTON       0x04u   /* response DATA = button state        */

/* Status codes */
#define STATUS_OK            0x00u
#define STATUS_BAD_CMD       0xE1u
#define STATUS_BAD_DATA      0xE2u

#define GPIO_BT              GPIOA
#define GPIO_PIN_BT          GPIO_PIN_0

/* Types ---------------------------------------------------------------------*/
typedef enum
{
  WAIT_SOF = 0,
  WAIT_CMD,
  WAIT_DATA
} RxState_t;

typedef struct
{
  uint8_t cmd;
  uint8_t data;
} Frame_t;

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

QueueHandle_t   QueueFrame;

static uint8_t    RxByte;
static RxState_t  RxState = WAIT_SOF;
static Frame_t    RxFrame;
static TickType_t LastByteTick;

static const uint16_t LedPins[4] = { GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15 };

/* Private function prototypes -----------------------------------------------*/
void LinkLayer( void * pvParameters );
static uint8_t DecodeFrame(const Frame_t *pFrame);
static void    SendFrame(uint8_t cmd, uint8_t data);


/**
  * @brief  The application entry point.
  * @retval int
  */
void run_apk(void)
{
  xUartMutex = xSemaphoreCreateMutex();

  /* Queue of complete frames : ISR -> LinkLayer */
  QueueFrame = xQueueCreate( QUEUE_FRAME_LEN, sizeof( Frame_t ) );

  xTaskCreate(LinkLayer, "LinkLayer", 256, NULL, 3, NULL);

  LOG_INFO(COLOR_WHITE," RTOS LAB APK5 (UART protocol) STARTING-----------------");

  /* Arm reception of the first byte */
  HAL_UART_Receive_IT(&huart2, &RxByte, 1);

  vTaskStartScheduler();

  while (1)
  {

  }
}


/* LinkLayer task : blocked on QueueFrame, decodes and answers each frame */
void LinkLayer(void *p)
{
  Frame_t frame;
  uint8_t rspData;

  while(1)
  {
    /* Blocked until the ISR posts a complete frame */
    xQueueReceive(QueueFrame, &frame, portMAX_DELAY);

    LOG_INFO(COLOR_CYAN, "RX frame CMD=0x%02X DATA=0x%02X", frame.cmd, frame.data);

    rspData = DecodeFrame(&frame);
    SendFrame(frame.cmd | FRAME_RSP_FLAG, rspData);
  }
}


/* Execute the command, return the DATA byte of the response */
static uint8_t DecodeFrame(const Frame_t *pFrame)
{
  switch (pFrame->cmd)
  {
    case CMD_PING:
      return pFrame->data;

    case CMD_LED_SET:
      for (uint8_t i = 0; i < 4; i++)
      {
        HAL_GPIO_WritePin(GPIOD, LedPins[i],
                          (pFrame->data & (1u << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
      return STATUS_OK;

    case CMD_LED_TOGGLE:
      if (pFrame->data > 3)
      {
        LOG_ERROR("LED index %u out of range", pFrame->data);
        return STATUS_BAD_DATA;
      }
      HAL_GPIO_TogglePin(GPIOD, LedPins[pFrame->data]);
      return STATUS_OK;

    case CMD_GET_BUTTON:
      return (HAL_GPIO_ReadPin(GPIO_BT, GPIO_PIN_BT) == GPIO_PIN_SET) ? 1u : 0u;

    default:
      LOG_ERROR("Unknown CMD 0x%02X", pFrame->cmd);
      return STATUS_BAD_CMD;
  }
}


/* Send a 3-byte frame, protected by the same mutex as printf */
static void SendFrame(uint8_t cmd, uint8_t data)
{
  uint8_t tx[3] = { FRAME_SOF, cmd, data };

  xSemaphoreTake(xUartMutex, portMAX_DELAY);
  HAL_UART_Transmit(&huart2, tx, sizeof(tx), 100);
  xSemaphoreGive(xUartMutex);
}


/* UART RX callback : byte-level state machine (ISR context) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriority = pdFALSE;
  TickType_t now;

  if (huart->Instance != USART2)
    return;

  /* Inter-byte timeout : drop a half-received frame */
  now = xTaskGetTickCountFromISR();
  if ((RxState != WAIT_SOF) && ((now - LastByteTick) > pdMS_TO_TICKS(FRAME_TIMEOUT_MS)))
  {
    RxState = WAIT_SOF;
  }
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
      /* Frame complete -> hand it to the LinkLayer task */
      xQueueSendFromISR(QueueFrame, &RxFrame, &xHigherPriority);
      RxState = WAIT_SOF;
      break;

    default:
      RxState = WAIT_SOF;
      break;
  }

  /* Re-arm reception of the next byte */
  HAL_UART_Receive_IT(&huart2, &RxByte, 1);

  /*if True force contex switch else wait for next tick*/
  portYIELD_FROM_ISR(xHigherPriority);
}


/* On UART error (overrun, noise, framing) restart reception from scratch */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
    return;

  RxState = WAIT_SOF;
  HAL_UART_Receive_IT(&huart2, &RxByte, 1);
}

#endif
