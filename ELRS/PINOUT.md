# ELRS Pinout — ESP32-C3 + SX1280

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         ESP32-C3 DevKit (TX / RX)                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌─────────────────────┐         ┌─────────────────────┐                   │
│   │      SX1280 Radio   │         │   CRSF UART         │                   │
│   │      (SPI + GPIO)   │         │   420000 baud       │                   │
│   └─────────┬───────────┘         └──────────┬──────────┘                   │
│             │                                │                              │
│   GPIO  7 ──┼── NSS (chip select)            │                              │
│   GPIO  6 ──┼── SCK                          │                              │
│   GPIO  5 ──┼── MISO                         │     TX: Computer → ESP32     │
│   GPIO 10 ──┼── MOSI                         │     RX: ESP32 → Flight Ctrl  │
│   GPIO  4 ──┼── DIO1 (interrupt)             │                              │
│   GPIO 18 ──┼── RST (reset)                  │   GPIO  0 ─── CRSF TX ───────┼──→ to FC RX / from PC TX
│   GPIO 19 ──┼── BUSY                         │   GPIO  1 ─── CRSF RX ───────┼──→ from FC TX / from PC RX
│             │                                │                              │
│   ┌─────────┴───────────┐         ┌──────────┴──────────┐                   │
│   │  3.3V  GND  ANT     │         │   (Serial1)         │                   │
│   └─────────────────────┘         └─────────────────────┘                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Pin assignment table

| GPIO | Function   | Direction     | Notes                          |
|------|------------|---------------|--------------------------------|
| 0    | CRSF TX    | Output        | ⚠️ Don’t hold low at boot       |
| 1    | CRSF RX    | Input         |                                |
| 4    | SX1280 DIO1| Input         | Interrupt / packet done        |
| 5    | SX1280 MISO| Input         | SPI data in                    |
| 6    | SX1280 SCK | Output        | SPI clock                      |
| 7    | SX1280 NSS | Output        | SPI chip select                |
| 10   | SX1280 MOSI| Output        | SPI data out                   |
| 18   | SX1280 RST | Output        | Radio reset                    |
| 19   | SX1280 BUSY| Input         | Radio busy flag                |

## Wiring reference

### SX1280 module → ESP32-C3

| SX1280 pin | ESP32-C3 | Notes                |
|------------|----------|----------------------|
| NSS (CS)   | GPIO 7   | SPI chip select      |
| SCK        | GPIO 6   | SPI clock            |
| MISO       | GPIO 5   | SPI master in        |
| MOSI       | GPIO 10  | SPI master out       |
| DIO1       | GPIO 4   | IRQ / packet event   |
| RST        | GPIO 18  | Active-low reset     |
| BUSY       | GPIO 19  | High = radio busy    |
| VCC        | 3.3V     | 2.4V–3.6V supply     |
| GND        | GND      | Common ground        |

### CRSF UART

| Role  | TX device          | RX device          |
|-------|--------------------|--------------------|
| **TX**| PC / flight controller | Computer UART RX | ESP32 GPIO 0 (CRSF TX) |
| **RX**| Computer UART TX   | ESP32 GPIO 1 (CRSF RX) | Flight controller TX |

- **TX build**: PC sends CRSF over UART → ESP32 GPIO 1 (RX) → RF out.
- **RX build**: RF in → ESP32 GPIO 0 (TX) → flight controller UART RX.

## ESP32-C3 available GPIOs

ESP32-C3 supports GPIOs: 0, 1, 4, 5, 6, 7, 8, 9, 10, 18, 19, 20, 21.  
This pinout uses only: **0, 1, 4, 5, 6, 7, 10, 18, 19**.

To change pins, edit `src/config.h`.
