/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : apk6.c
  * @brief          : APK6 - Washing machine application on top of the APK5 link
  ******************************************************************************
  *
  *  Frame format (3 bytes) :   | SOF 0xAA | CMD | DATA |
  *
  *  Host -> STM32 (requests, answered with CMD|0x80 + status)
  *     0x01 PING          data echoed
  *     0x10 CFG_PROGRAM   0 Cotton, 1 Synthetic, 2 Wool, 3 Quick
  *     0x11 CFG_TEMP      20, 30, 40, 60 or 90 (deg C)
  *     0x12 CFG_SPIN      rpm / 100, 4..14
  *     0x20 START_CYCLE   data ignored
  *     0x21 GET_STATUS    answer data = system state (0 IDLE, 1 READY, 2 RUN)
  *                        + bit7 set when the sensor is in ERROR_STATE (door open)
  *
  *  STM32 -> Host (events, not answers)
  *     0x30 EVT_STEP      data = step 1..6 entered
  *     0x31 EVT_PROGRESS  data = progress of the current step 0..100 %
  *     0x32 EVT_DONE      cycle finished
  *     0x33 EVT_DOOR      data = 1 open, 0 closed
  *     0x34 EVT_ERROR     data = error code (0x01 door open during cycle)
  *
  *  Tasks :
  *     LinkLayer  (prio 3) : decodes frames, answers PING / GET_STATUS,
  *                           rejects application commands while a cycle runs,
  *                           forwards the others to SystemTask through QueueCmd
  *     SystemTask (prio 2) : state machine
  *                             IDLE_STATE  : wait for a complete configuration
  *                             READY_STATE : configuration valid, wait for START
  *                             RUN_STATE   : 6 steps, EVT_STEP / EVT_PROGRESS,
  *                                           EVT_DONE, back to READY_STATE
  *     SensorTask (prio 4) : state machine on the user button (door)
  *                             NO_ERROR_STATE : door closed; press -> door open,
  *                                              suspend SystemTask if running
  *                             ERROR_STATE    : door open; press -> door closed,
  *                                              resume SystemTask if suspended
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

#ifdef APK6

/* Protocol ------------------------------------------------------------------*/
#define FRAME_SOF            0xAAu
#define FRAME_RSP_FLAG       0x80u
#define FRAME_TIMEOUT_MS     50u

#define CMD_PING             0x01u
#define CMD_CFG_PROGRAM      0x10u
#define CMD_CFG_TEMP         0x11u
#define CMD_CFG_SPIN         0x12u
#define CMD_START_CYCLE      0x20u
#define CMD_GET_STATUS       0x21u

#define EVT_STEP             0x30u
#define EVT_PROGRESS         0x31u
#define EVT_DONE             0x32u
#define EVT_DOOR             0x33u
#define EVT_ERROR            0x34u

#define STATUS_OK            0x00u
#define STATUS_BAD_CMD       0xE1u
#define STATUS_BAD_DATA      0xE2u
#define STATUS_BUSY          0xE3u   /* cycle running: no config, no start */
#define STATUS_NOT_CONFIG    0xE4u   /* start without a complete config    */
#define STATUS_DOOR_OPEN     0xE5u   /* start with the door open           */
#define STATUS_ERROR_FLAG    0x80u   /* GET_STATUS: sensor in ERROR_STATE   */

#define ERR_DOOR_OPEN        0x01u

/* Application ---------------------------------------------------------------*/
#define NB_STEPS             6u
#define STEP_TIME_MS         10000u  /* duration of one step               */
#define TICK_MS              100u    /* SystemTask period in RUN           */
#define PROGRESS_EVERY       2u      /* EVT_PROGRESS every 2 ticks = 200 ms */
#define DEBOUNCE_MS          30u

#define CFG_PROGRAM_BIT      (1u << 0)
#define CFG_TEMP_BIT         (1u << 1)
#define CFG_SPIN_BIT         (1u << 2)
#define CFG_COMPLETE         (CFG_PROGRAM_BIT | CFG_TEMP_BIT | CFG_SPIN_BIT)

#define GPIO_BT              GPIOA
#define GPIO_PIN_BT          GPIO_PIN_0
#define LED_RUN              GPIO_PIN_12   /* green : blinks while running */
#define LED_CONFIG           GPIO_PIN_13   /* orange: configuration valid  */
#define LED_DOOR             GPIO_PIN_14   /* red   : door open            */
#define LED_DONE             GPIO_PIN_15   /* blue  : cycle finished       */

/* Types ---------------------------------------------------------------------*/
typedef enum { WAIT_SOF = 0, WAIT_CMD, WAIT_DATA } RxState_t;
typedef enum { IDLE_STATE = 0, READY_STATE, RUN_STATE } SysState_t;
typedef enum { NO_ERROR_STATE = 0, ERROR_STATE } SensorState_t;

typedef struct
{
  uint8_t cmd;
  uint8_t data;
} Frame_t;

typedef struct
{
  uint8_t program;
  uint8_t temp;
  uint8_t spin;
  uint8_t mask;          /* CFG_x_BIT received */
} Config_t;

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

QueueHandle_t       QueueFrame;   /* ISR        -> LinkLayer  */
QueueHandle_t       QueueCmd;     /* LinkLayer  -> SystemTask */
SemaphoreHandle_t   SemDoor;      /* EXTI       -> SensorTask */
TaskHandle_t        hSystemTask;

/* Each state has a single writer: SysState = SystemTask, SensorState = SensorTask */
static volatile SysState_t    SysState    = IDLE_STATE;
static volatile SensorState_t SensorState = NO_ERROR_STATE;

static uint8_t    RxByte;
static RxState_t  RxState = WAIT_SOF;
static Frame_t    RxFrame;
static TickType_t LastByteTick;

static const char *ProgramName[] = { "Cotton", "Synthetic", "Wool", "Quick" };
static const char *StepName[NB_STEPS] = { "Fill", "Heat", "Wash", "Rinse", "Drain", "Spin" };
static const uint8_t ValidTemp[] = { 20, 30, 40, 60, 90 };

/* Private function prototypes -----------------------------------------------*/
void LinkLayer( void * pvParameters );
void SystemTask( void * pvParameters );
void SensorTask( void * pvParameters );
static void    SendFrame(uint8_t cmd, uint8_t data);
static uint8_t ApplyConfig(Config_t *pCfg, const Frame_t *pFrame);
static void    RunCycle(void);


/**
  * @brief  The application entry point.
  */
void run_apk(void)
{
  xUartMutex = xSemaphoreCreateMutex();
  SemDoor    = xSemaphoreCreateBinary();
  QueueFrame = xQueueCreate( 8, sizeof( Frame_t ) );
  QueueCmd   = xQueueCreate( 4, sizeof( Frame_t ) );

  xTaskCreate(LinkLayer,  "LinkLayer",  256, NULL, 3, NULL);
  xTaskCreate(SystemTask, "System",     256, NULL, 2, &hSystemTask);
  xTaskCreate(SensorTask, "Sensor",     256, NULL, 4, NULL);

  LOG_INFO(COLOR_WHITE," RTOS LAB APK6 (washing machine) STARTING-----------------");

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

  while(1)
  {
    xQueueReceive(QueueFrame, &frame, portMAX_DELAY);
    LOG_INFO(COLOR_CYAN, "RX frame CMD=0x%02X DATA=0x%02X", frame.cmd, frame.data);

    switch (frame.cmd)
    {
      /* Link level : answered here */
      case CMD_PING:
        SendFrame(frame.cmd | FRAME_RSP_FLAG, frame.data);
        break;

      case CMD_GET_STATUS:
        SendFrame(frame.cmd | FRAME_RSP_FLAG,
                  (uint8_t)SysState | ((SensorState == ERROR_STATE) ? STATUS_ERROR_FLAG : 0u));
        break;

      /* Application level : only the SystemTask in IDLE may handle them */
      case CMD_CFG_PROGRAM:
      case CMD_CFG_TEMP:
      case CMD_CFG_SPIN:
      case CMD_START_CYCLE:
        if (SysState == RUN_STATE)      /* also while suspended by the door */
        {
          LOG_ERROR("Cycle running: CMD 0x%02X refused", frame.cmd);
          SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_BUSY);
        }
        else if (xQueueSend(QueueCmd, &frame, 0) != pdPASS)
        {
          SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_BUSY);
        }
        break;

      default:
        LOG_ERROR("Unknown CMD 0x%02X", frame.cmd);
        SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_BAD_CMD);
        break;
    }
  }
}


/* ====================================================== System task =======*/
void SystemTask(void *p)
{
  Config_t cfg = { 0 };
  Frame_t  frame;
  uint8_t  status;

  while(1)
  {
    switch (SysState)
    {
      /* ---- IDLE_STATE : wait for a complete configuration ---- */
      case IDLE_STATE:
        xQueueReceive(QueueCmd, &frame, portMAX_DELAY);
        if (frame.cmd == CMD_START_CYCLE)
        {
          LOG_ERROR("Start refused: no configuration");
          SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_NOT_CONFIG);
          break;
        }
        status = ApplyConfig(&cfg, &frame);
        SendFrame(frame.cmd | FRAME_RSP_FLAG, status);
        if (cfg.mask == CFG_COMPLETE)
        {
          HAL_GPIO_WritePin(GPIOD, LED_CONFIG, GPIO_PIN_SET);
          LOG_INFO(COLOR_GREEN, "Configured: %s, %uC, %u rpm -> READY_STATE",
                   ProgramName[cfg.program], cfg.temp, cfg.spin * 100u);
          SysState = READY_STATE;
        }
        break;

      /* ---- READY_STATE : configuration valid, wait for START ---- */
      case READY_STATE:
        xQueueReceive(QueueCmd, &frame, portMAX_DELAY);
        if (frame.cmd != CMD_START_CYCLE)
        {
          /* a new parameter replaces the old one, the config stays complete */
          SendFrame(frame.cmd | FRAME_RSP_FLAG, ApplyConfig(&cfg, &frame));
          break;
        }
        if (SensorState == ERROR_STATE)
        {
          LOG_ERROR("Start refused: door open");
          SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_DOOR_OPEN);
          break;
        }
        SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_OK);
        LOG_INFO(COLOR_GREEN, "Cycle start: %s, %uC, %u rpm -> RUN_STATE",
                 ProgramName[cfg.program], cfg.temp, cfg.spin * 100u);
        HAL_GPIO_WritePin(GPIOD, LED_DONE, GPIO_PIN_RESET);
        SysState = RUN_STATE;
        break;

      /* ---- RUN_STATE : execute the cycle (may be suspended by SensorTask) ---- */
      case RUN_STATE:
        RunCycle();
        HAL_GPIO_WritePin(GPIOD, LED_RUN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LED_DONE, GPIO_PIN_SET);
        LOG_INFO(COLOR_GREEN, "Cycle done -> READY_STATE");
        SysState = READY_STATE;           /* configuration is kept */
        SendFrame(EVT_DONE, 0);
        break;

      default:
        SysState = IDLE_STATE;
        break;
    }
  }
}


/* Validate and store one configuration parameter */
static uint8_t ApplyConfig(Config_t *pCfg, const Frame_t *pFrame)
{
  switch (pFrame->cmd)
  {
    case CMD_CFG_PROGRAM:
      if (pFrame->data > 3) return STATUS_BAD_DATA;
      pCfg->program = pFrame->data;
      pCfg->mask |= CFG_PROGRAM_BIT;
      return STATUS_OK;

    case CMD_CFG_TEMP:
      for (uint8_t i = 0; i < sizeof(ValidTemp); i++)
      {
        if (ValidTemp[i] == pFrame->data)
        {
          pCfg->temp = pFrame->data;
          pCfg->mask |= CFG_TEMP_BIT;
          return STATUS_OK;
        }
      }
      return STATUS_BAD_DATA;

    case CMD_CFG_SPIN:
      if ((pFrame->data < 4) || (pFrame->data > 14)) return STATUS_BAD_DATA;
      pCfg->spin = pFrame->data;
      pCfg->mask |= CFG_SPIN_BIT;
      return STATUS_OK;

    default:
      return STATUS_BAD_CMD;
  }
}


/* Run the 6 steps. Time only advances while the task runs, so a
   suspension (door open) freezes the cycle where it is. */
static void RunCycle(void)
{
  const uint32_t ticksPerStep = STEP_TIME_MS / TICK_MS;
  Frame_t frame;

  for (uint8_t step = 1; step <= NB_STEPS; step++)
  {
    SendFrame(EVT_STEP, step);
    LOG_INFO(COLOR_YELLOW, "Step %u: %s", step, StepName[step - 1]);

    for (uint32_t tick = 1; tick <= ticksPerStep; tick++)
    {
      vTaskDelay(pdMS_TO_TICKS(TICK_MS));

      /* A command queued just before RUN started: refuse it */
      while (xQueueReceive(QueueCmd, &frame, 0) == pdPASS)
        SendFrame(frame.cmd | FRAME_RSP_FLAG, STATUS_BUSY);

      if ((tick % PROGRESS_EVERY) == 0)
      {
        SendFrame(EVT_PROGRESS, (uint8_t)(tick * 100u / ticksPerStep));
        HAL_GPIO_TogglePin(GPIOD, LED_RUN);
      }
    }
  }
}


/* ====================================================== Sensor task =======*/
/* Wait for one debounced press of B1 (rising edge confirmed after 30 ms) */
static void WaitButtonPress(void)
{
  while(1)
  {
    xSemaphoreTake(SemDoor, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    if (HAL_GPIO_ReadPin(GPIO_BT, GPIO_PIN_BT) == GPIO_PIN_SET)
      break;
    (void)xSemaphoreTake(SemDoor, 0);
  }

  /* wait for release, then drop the edges caused by the bounce */
  while (HAL_GPIO_ReadPin(GPIO_BT, GPIO_PIN_BT) == GPIO_PIN_SET)
    vTaskDelay(pdMS_TO_TICKS(10));
  vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
  (void)xSemaphoreTake(SemDoor, 0);
}


void SensorTask(void *p)
{
  uint8_t suspended = 0;     /* 1 = this task suspended the SystemTask */

  while(1)
  {
    WaitButtonPress();

    switch (SensorState)
    {
      /* ---- NO_ERROR_STATE : door closed, a press opens it ---- */
      case NO_ERROR_STATE:
        HAL_GPIO_WritePin(GPIOD, LED_DOOR, GPIO_PIN_SET);

        /* Take the UART mutex first: SystemTask is then sure not to own it,
           so suspending it cannot block the UART for everybody else. */
        xSemaphoreTake(xUartMutex, portMAX_DELAY);
        if (SysState == RUN_STATE)
        {
          vTaskSuspend(hSystemTask);
          suspended = 1;
        }
        xSemaphoreGive(xUartMutex);

        SendFrame(EVT_DOOR, 1);
        if (suspended)
        {
          SendFrame(EVT_ERROR, ERR_DOOR_OPEN);
          LOG_ERROR("Door open: cycle suspended -> ERROR_STATE");
        }
        SensorState = ERROR_STATE;
        break;

      /* ---- ERROR_STATE : door open, a press closes it ---- */
      case ERROR_STATE:
        HAL_GPIO_WritePin(GPIOD, LED_DOOR, GPIO_PIN_RESET);
        SensorState = NO_ERROR_STATE;
        SendFrame(EVT_DOOR, 0);
        if (suspended)
        {
          suspended = 0;
          LOG_INFO(COLOR_GREEN, "Door closed: cycle resumed -> NO_ERROR_STATE");
          vTaskResume(hSystemTask);
        }
        break;

      default:
        SensorState = NO_ERROR_STATE;
        break;
    }
  }
}


/* ====================================================== Helpers / ISR =====*/
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


/* User button = door sensor */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriority = pdFALSE;

  if (GPIO_Pin != GPIO_PIN_BT)
    return;

  xSemaphoreGiveFromISR(SemDoor, &xHigherPriority);
  portYIELD_FROM_ISR(xHigherPriority);
}

#endif
