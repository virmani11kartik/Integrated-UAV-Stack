// Ebyte E28-2G4T12S UART radio wrapper (transparent transmission)
#pragma once

#include "config.h"
#include <Arduino.h>

#if ELRS_USE_E28
class RadioE28Uart {
public:
    RadioE28Uart();
    bool begin(const uint8_t* uid);
    void setFrequency(uint32_t regFreq);
    void transmit(uint8_t* data, uint8_t len);
    void startReceive();
    bool readPacket(uint8_t* data, uint8_t len);
    bool isTxDone();
    bool isRxDone();
    void clearIrq();

private:
    HardwareSerial* serial_;
};

extern RadioE28Uart Radio;
#endif
