/**
  ******************************************************************************
  * @file    apk10_logic.c
  * @brief   APK10 decision logic (TDD step 2: GREEN).
  *          Written to make tests/apk10/test_apk10_logic.c pass.
  *          Pure C: no HAL, no FreeRTOS -> unit-tested on the PC.
  ******************************************************************************
  */
#include "apk10_logic.h"

/* SW-REQ-003 */
uint8_t Apk10_Thermostat(uint8_t temp, uint8_t lastPower)
{
  if (temp > APK10_SENSOR_MAX_C)                     return 0;     /* unknown T: do not heat */
  if (temp + APK10_HYSTERESIS_C <= APK10_SET_POINT_C) return 100;
  if (temp >= APK10_SET_POINT_C)                     return 0;
  return lastPower;                                               /* hysteresis band */
}

/* SW-REQ-007..010 */
uint8_t Apk10_CheckErrors(const Apk10_Inputs_t *in)
{
  uint8_t err = 0;

  if (in->sensorSeen)                               /* sensor supervision starts with the 1st sample */
  {
    if (in->sensorValue > APK10_SENSOR_MAX_C)        err |= APK10_ERR_SENSOR;
    else if (in->sensorValue > APK10_OVERTEMP_C)     err |= APK10_ERR_OVERTEMP;
    if (in->sensorAgeMs > APK10_SENSOR_TIMEOUT_MS)   err |= APK10_ERR_TIMEOUT;
  }
  if (in->aliveStillMs > APK10_ALIVE_TIMEOUT_MS)     err |= APK10_ERR_ALIVE;
  return err;
}

/* SW-REQ-012..014 */
Apk10_Step_t Apk10_SafetyStep(Apk10Mode_t mode, uint8_t err, uint8_t powerRequest)
{
  Apk10_Step_t s = { APK10_SAFE, 0, 0, 0 };           /* default = the safe state */

  if (mode == APK10_NORMAL)
  {
    if (err == 0)
    {
      s.mode        = APK10_NORMAL;
      s.power       = powerRequest;
      s.relayClosed = 1;
    }
    else
    {
      s.enteredSafe = 1;
    }
  }
  return s;                                         /* SAFE: stays SAFE, outputs off */
}

/* SW-REQ-015 */
uint8_t Apk10_ResetAllowed(uint8_t sensorSeen, uint8_t sensorValue, uint32_t sensorAgeMs)
{
  return (uint8_t)(sensorSeen &&
                   (sensorAgeMs < APK10_SENSOR_TIMEOUT_MS) &&
                   (sensorValue < APK10_RESET_MAX_TEMP_C));
}

/* SW-REQ-001 */
void Apk10_RxInit(Apk10_Rx_t *rx)
{
  rx->state  = 0;
  rx->cmd    = 0;
  rx->lastMs = 0;
}

uint8_t Apk10_RxByte(Apk10_Rx_t *rx, uint8_t byte, uint32_t nowMs, uint8_t *cmd, uint8_t *data)
{
  uint8_t done = 0;

  if ((rx->state != 0) && ((nowMs - rx->lastMs) > APK10_FRAME_TIMEOUT_MS))
    rx->state = 0;                                  /* partial frame dropped */
  rx->lastMs = nowMs;

  switch (rx->state)
  {
    case 0:                                         /* WAIT_SOF  */
      if (byte == APK10_FRAME_SOF)
        rx->state = 1;
      break;
    case 1:                                         /* WAIT_CMD  */
      rx->cmd   = byte;
      rx->state = 2;
      break;
    default:                                        /* WAIT_DATA */
      *cmd      = rx->cmd;
      *data     = byte;
      rx->state = 0;
      done      = 1;
      break;
  }
  return done;
}
