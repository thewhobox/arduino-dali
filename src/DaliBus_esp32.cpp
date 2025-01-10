#pragma once

#include "DaliBus.h"
#ifdef DALI_USE_ESP32
#include "arduino.h"

void DaliBusClass::begin(byte tx_pin, byte rx_pin, bool active_low)
{

}

daliReturnValue DaliBusClass::sendRaw(const byte * message, uint8_t bits)
{
    return DALI_NO_ERROR;
}

int DaliBusClass::getLastResponse()
{
    return 0;
}

bool DaliBusClass::busIsIdle()
{
    return true;
}
#endif