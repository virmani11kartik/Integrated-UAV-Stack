// ELRS config for ESP32-C3 (TX and RX)
#pragma once

#include <Arduino.h>

// ------------------------- Bind phrase / UID -------------------------
// Option A (phrase bind): Same phrase on TX and RX. Max 6 bytes used for UID.
// Option B (classic bind): TX broadcasts UID, RX auto-binds. Set ELRS_CLASSIC_BIND=1.
#define ELRS_BIND_PHRASE "elrsuav"
#define ELRS_UID_LEN 6
#define ELRS_CLASSIC_BIND 1   // 1 = broadcast & auto-bind; 0 = use bind phrase
#define ELRS_BIND_INTERVAL_PACKETS 25  // TX sends BIND every N packets (classic bind only)

// ------------------------- Radio selection -------------------------
// 0 = SX1280 (SPI), 1 = Ebyte E28-2G4T12S (UART)
#define ELRS_USE_E28 1

#if ELRS_USE_E28
// ------------------------- E28-2G4T12S (UART) -------------------------
// Uses transparent transmission. Configure parameters on boot if needed.
// UART port: 0 = Serial, 1 = Serial1. Do not share with CRSF.
#define E28_UART_PORT 0
// Keep E28_UART_BAUD consistent with E28_UART_BAUD_CODE.
#define E28_UART_BAUD 460800
// 0=8N1, 1=8O1, 2=8E1 (must match module config)
#define E28_UART_PARITY 0
// E28 config bytes (see datasheet): SPED = parity(7:6) | uart baud(5:3) | air rate(2:0)
#define E28_UART_BAUD_CODE 0x06  // 0x06 = 460800
#define E28_AIR_RATE_CODE  0x06  // 0x06 = 1.0 Mbps (FLRC)
#define E28_ADDR_H 0x00
#define E28_ADDR_L 0x00
#define E28_CHANNEL 0x18  // 0x00-0xFF
// OPTION bits: bit7 fixed, bit3 LBT, bit2 IO drive, bit1:0 power (0=12 dBm)
#define E28_OPTION_FIXED   0
#define E28_OPTION_LBT     0
#define E28_OPTION_IO_DRIVE 1
#define E28_OPTION_TX_POWER 0
#define E28_CONFIG_ON_BOOT 1
#define E28_AUX_WAIT_MS    200

// E28 UART pins (ESP32-C3). Set to -1 to use default UART pins.
#define E28_PIN_TX  4
#define E28_PIN_RX  5
#define E28_PIN_M0  6
#define E28_PIN_M1  7
#define E28_PIN_M2  10
#define E28_PIN_AUX 18

#define E28_SPED   ((uint8_t)((E28_UART_PARITY << 6) | (E28_UART_BAUD_CODE << 3) | (E28_AIR_RATE_CODE)))
#define E28_OPTION ((uint8_t)((E28_OPTION_FIXED << 7) | (E28_OPTION_LBT << 3) | (E28_OPTION_IO_DRIVE << 2) | (E28_OPTION_TX_POWER)))
#else
// ------------------------- Radio (SX1280) pins -------------------------
// ESP32-C3 available GPIOs only: 0, 1, 4, 5, 6, 7, 10, 18, 19
#define RADIO_PIN_NSS   7
#define RADIO_PIN_RST   18
#define RADIO_PIN_BUSY  19
#define RADIO_PIN_DIO1  4
#define RADIO_PIN_SCK   6
#define RADIO_PIN_MISO  5
#define RADIO_PIN_MOSI  10
#endif

// ------------------------- UART (CRSF) -------------------------
// TX: computer sends CRSF here. RX: output CRSF to flight controller.
// Uses GPIO 0, 1 (Serial1). Avoid holding GPIO 0 low at boot (bootloader).
#define CRSF_UART_PORT 1
#define CRSF_UART_BAUD 420000
#define CRSF_PIN_TX    0
#define CRSF_PIN_RX    1

static inline HardwareSerial* GetSerialByPort(uint8_t port) {
    return (port == 0) ? &Serial : &Serial1;
}

// ------------------------- Logging -------------------------
// Set to -1 to disable logs when UARTs are in use.
#if ELRS_USE_E28
#define ELRS_LOG_PORT -1
#else
#define ELRS_LOG_PORT 0
#endif

static inline Print* GetLogSerial() {
#if ELRS_LOG_PORT == 0
    return &Serial;
#elif ELRS_LOG_PORT == 1
    return &Serial1;
#else
    return nullptr;
#endif
}

// ------------------------- Rate -------------------------
// Packet interval in microseconds (500 Hz = 2000 us)
#define ELRS_PACKET_INTERVAL_US  2000
#define ELRS_FHSS_HOP_INTERVAL   2     // hop every N packets
#define ELRS_PACKET_SIZE         8     // OTA4 = 8 bytes

// ------------------------- 2.4 GHz FHSS -------------------------
// ISM 2.4 GHz band (same as ExpressLRS)
#define ELRS_FREQ_START_HZ  2400400000UL
#define ELRS_FREQ_END_HZ    2479400000UL
#define ELRS_FREQ_COUNT     80
#define ELRS_FHSS_SEQ_LEN   256

// ------------------------- CRSF channel range -------------------------
#define CRSF_CHANNEL_VALUE_MIN 172
#define CRSF_CHANNEL_VALUE_MID 992
#define CRSF_CHANNEL_VALUE_MAX 1811
#define CRSF_NUM_CHANNELS      16
