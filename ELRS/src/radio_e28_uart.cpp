#include "radio_e28_uart.h"

#if ELRS_USE_E28

RadioE28Uart Radio;

static inline HardwareSerial* GetUart(uint8_t port) {
    return (port == 0) ? &Serial : &Serial1;
}

static inline void PinModeIfValid(int8_t pin, uint8_t mode) {
    if (pin >= 0) pinMode(pin, mode);
}

static inline void DigitalWriteIfValid(int8_t pin, uint8_t val) {
    if (pin >= 0) digitalWrite(pin, val);
}

static bool WaitAuxHigh(uint32_t timeoutMs) {
    if (E28_PIN_AUX < 0) return true;
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (digitalRead(E28_PIN_AUX) == HIGH) return true;
        delay(1);
    }
    return false;
}

static uint32_t E28UartConfigNormal() {
    switch (E28_UART_PARITY) {
        case 1: return SERIAL_8O1;
        case 2: return SERIAL_8E1;
        default: return SERIAL_8N1;
    }
}

static void BeginUart(HardwareSerial* ser, uint32_t baud, uint32_t config) {
    if (E28_PIN_RX >= 0 && E28_PIN_TX >= 0)
        ser->begin(baud, config, E28_PIN_RX, E28_PIN_TX);
    else
        ser->begin(baud, config);
}

static void SetModePins(bool m0, bool m1, bool m2) {
    DigitalWriteIfValid(E28_PIN_M0, m0 ? HIGH : LOW);
    DigitalWriteIfValid(E28_PIN_M1, m1 ? HIGH : LOW);
    DigitalWriteIfValid(E28_PIN_M2, m2 ? HIGH : LOW);
}

static void ConfigureModule(HardwareSerial* ser) {
#if E28_CONFIG_ON_BOOT
    // Configuration mode: M0=1, M1=1, M2=1 per E28 datasheet.
    SetModePins(true, true, true);
    WaitAuxHigh(E28_AUX_WAIT_MS);
    BeginUart(ser, 9600, SERIAL_8N1);
    delay(40);

    uint8_t cmd[6] = {0xC0, E28_ADDR_H, E28_ADDR_L, E28_SPED, E28_CHANNEL, E28_OPTION};
    ser->write(cmd, sizeof(cmd));
    ser->flush();
    delay(40);
    while (ser->available()) (void)ser->read();

    // Normal transmission mode: M0=0, M1=0, M2=1.
    SetModePins(false, false, true);
    WaitAuxHigh(E28_AUX_WAIT_MS);
#endif
}

RadioE28Uart::RadioE28Uart() : serial_(nullptr) {}

bool RadioE28Uart::begin(const uint8_t* uid) {
    (void)uid;
    serial_ = GetUart(E28_UART_PORT);

    PinModeIfValid(E28_PIN_M0, OUTPUT);
    PinModeIfValid(E28_PIN_M1, OUTPUT);
    PinModeIfValid(E28_PIN_M2, OUTPUT);
    PinModeIfValid(E28_PIN_AUX, INPUT);

    ConfigureModule(serial_);
#if !E28_CONFIG_ON_BOOT
    // Ensure normal transmission mode when not configuring on boot.
    SetModePins(false, false, true);
    WaitAuxHigh(E28_AUX_WAIT_MS);
#endif

    // Normal working mode with configured UART settings.
    BeginUart(serial_, E28_UART_BAUD, E28UartConfigNormal());
    return true;
}

void RadioE28Uart::setFrequency(uint32_t regFreq) {
    (void)regFreq;  // Fixed channel on E28; set via config.
}

void RadioE28Uart::transmit(uint8_t* data, uint8_t len) {
    if (!serial_) return;
    if (serial_->availableForWrite() < len) return;
    serial_->write(data, len);
}

void RadioE28Uart::startReceive() {
    // No-op for UART stream.
}

bool RadioE28Uart::readPacket(uint8_t* data, uint8_t len) {
    if (!serial_) return false;
    if (serial_->available() < len) return false;
    for (uint8_t i = 0; i < len; i++) {
        int v = serial_->read();
        if (v < 0) return false;
        data[i] = (uint8_t)v;
    }
    return true;
}

bool RadioE28Uart::isTxDone() {
    return true;
}

bool RadioE28Uart::isRxDone() {
    if (!serial_) return false;
    return serial_->available() >= ELRS_PACKET_SIZE;
}

void RadioE28Uart::clearIrq() {
}

#endif
