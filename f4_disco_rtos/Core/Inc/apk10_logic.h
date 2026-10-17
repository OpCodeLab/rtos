/**
  ******************************************************************************
  * @file    apk10_logic.h
  * @brief   APK10 decision logic, written test-first (TDD).
  *
  *  Pure C: no HAL, no FreeRTOS, no global state. Every function takes its
  *  inputs as parameters and returns its decision, so it can be unit-tested
  *  on the PC (tests/apk10) before it ever runs on the STM32.
  *  apk10.c only collects the inputs, calls these functions and applies the
  *  result to the hardware.
  ******************************************************************************
  */
#ifndef APK10_LOGIC_H
#define APK10_LOGIC_H

#include <stdint.h>

/* Requirements values (docs/APK10_Requirements_ASPICE.xlsx) */
#define APK10_SET_POINT_C        40u
#define APK10_HYSTERESIS_C       2u
#define APK10_SENSOR_MAX_C       120u
#define APK10_OVERTEMP_C         90u
#define APK10_SENSOR_TIMEOUT_MS  500u
#define APK10_ALIVE_TIMEOUT_MS   300u
#define APK10_RESET_MAX_TEMP_C   60u
#define APK10_FRAME_SOF          0xAAu
#define APK10_FRAME_TIMEOUT_MS   50u

#define APK10_ERR_SENSOR         0x01u
#define APK10_ERR_TIMEOUT        0x02u
#define APK10_ERR_OVERTEMP       0x04u
#define APK10_ERR_ALIVE          0x08u

typedef enum { APK10_NORMAL = 0, APK10_SAFE = 1 } Apk10Mode_t;

/* SW-REQ-003: thermostat. Returns the power the function wants (0 or 100). */
uint8_t Apk10_Thermostat(uint8_t temp, uint8_t lastPower);

/* SW-REQ-007..010: the 4 errors. Returns a bit mask, 0 = no error. */
typedef struct
{
  uint8_t  sensorSeen;     /* at least one SENSOR_TEMP received        */
  uint8_t  sensorValue;    /* last value, 0xFF = broken sensor         */
  uint32_t sensorAgeMs;    /* time since the last SENSOR_TEMP          */
  uint32_t aliveStillMs;   /* time since ControlAlive last changed     */
} Apk10_Inputs_t;

uint8_t Apk10_CheckErrors(const Apk10_Inputs_t *in);

/* SW-REQ-012..014: one SafetyTask cycle. Decides the mode and the outputs. */
typedef struct
{
  Apk10Mode_t mode;         /* mode after this cycle                    */
  uint8_t    power;        /* heater power to write                    */
  uint8_t    relayClosed;  /* relay state to write                     */
  uint8_t    enteredSafe;  /* 1 = NORMAL -> SAFE in this cycle         */
} Apk10_Step_t;

Apk10_Step_t Apk10_SafetyStep(Apk10Mode_t mode, uint8_t err, uint8_t powerRequest);

/* SW-REQ-015: may the system leave SAFE? */
uint8_t Apk10_ResetAllowed(uint8_t sensorSeen, uint8_t sensorValue, uint32_t sensorAgeMs);

/* SW-REQ-001: RX frame parser (AA CMD DATA), one byte at a time. */
typedef struct
{
  uint8_t  state;          /* 0 WAIT_SOF, 1 WAIT_CMD, 2 WAIT_DATA      */
  uint8_t  cmd;
  uint32_t lastMs;
} Apk10_Rx_t;

void    Apk10_RxInit(Apk10_Rx_t *rx);
/* Returns 1 when a full frame is complete (cmd / data filled), else 0. */
uint8_t Apk10_RxByte(Apk10_Rx_t *rx, uint8_t byte, uint32_t nowMs, uint8_t *cmd, uint8_t *data);

#endif /* APK10_LOGIC_H */
