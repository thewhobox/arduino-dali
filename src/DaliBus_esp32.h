#pragma once

#include "DaliBus.h"
#ifdef DALI_USE_ESP32
#include "arduino.h"

class DaliBusClass {
  public:
    void begin(byte tx_pin, byte rx_pin, bool active_low = true);
    daliReturnValue sendRaw(const byte * message, uint8_t bits);

    int getLastResponse();

    bool busIsIdle();
    volatile byte busIdleCount;

    EventHandlerReceivedDataFuncPtr receivedCallback;
    EventHandlerActivityFuncPtr activityCallback;
    EventHandlerErrorFuncPtr errorCallback;

    //TODO remove temp
    bool tempBusLevel = false;
    uint16_t tempDelta = 0;
};

#endif