#pragma once

#include "arduino.h"

/** some enum */
typedef enum daliReturnValue {
  DALI_NO_ERROR = 0,
  DALI_RX_EMPTY = -1,
  DALI_RX_ERROR = -2,
  DALI_SENT = -3,
  DALI_INVALID_PARAMETER = -4,
  DALI_BUSY = -5,
  DALI_READY_TIMEOUT = -6,
  DALI_SEND_TIMEOUT = -7,
  DALI_COLLISION = -8,
  DALI_PULLDOWN = -9,
  DALI_CANT_BE_HIGH = -10,
  DALI_INVALID_STARTBIT = -11,
  DALI_ERROR_TIMING = -12,
  DALI_TX_ERROR = -13,
} daliReturnValue;

typedef void (*EventHandlerReceivedDataFuncPtr)(uint8_t *data, uint8_t bits);
typedef void (*EventHandlerActivityFuncPtr)();
typedef void (*EventHandlerErrorFuncPtr)(daliReturnValue errorCode);

#define DALI_ERR_CREATE_TX -1
#define DALI_ERR_CREATE_RX -2
#define DALI_ERR_ENABLE_TX -3
#define DALI_ERR_ENABLE_RX -4
#define DALI_ERR_CREATE_ENCODER -5


#ifdef DALI_USE_GENERIC
    #include "DaliBus_generic.h"
#elif defined(ARDUINO_ARCH_ESP32)
    #define DALI_USE_ESP32
    #include "DaliBus_esp32.h"
#else
    #define DALI_USE_GENERIC
    #include "DaliBus_generic.h"
#endif