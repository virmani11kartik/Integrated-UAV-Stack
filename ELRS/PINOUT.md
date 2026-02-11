# ELRS Pinout - ESP32-C3 + E28-2G4T12S

## Pin assignment table

| GPIO | Function   | Direction | Notes                              |
|------|------------|-----------|------------------------------------|
| 0    | CRSF TX    | Output    | Do not hold low at boot            |
| 1    | CRSF RX    | Input     |                                    |
| 4    | E28 TX     | Output    | ESP32 -> E28 RXD                   |
| 5    | E28 RX     | Input     | E28 TXD -> ESP32                   |
| 6    | E28 M0     | Output    | Mode select                        |
| 7    | E28 M1     | Output    | Mode select                        |
| 10   | E28 M2     | Output    | Mode select                        |
| 18   | E28 AUX    | Input     | Busy/status, high = idle           |

## Wiring reference

### E28-2G4T12S module -> ESP32-C3

| E28 pin | ESP32-C3 | Notes                            |
|---------|----------|----------------------------------|
| VCC     | 3.3V     | 3.3V supply                      |
| GND     | GND      | Common ground                    |
| TXD     | GPIO 5   | E28 TX -> ESP32 RX               |
| RXD     | GPIO 4   | ESP32 TX -> E28 RX               |
| M0      | GPIO 6   | Mode select                      |
| M1      | GPIO 7   | Mode select                      |
| M2      | GPIO 10  | Mode select                      |
| AUX     | GPIO 18  | Busy/status, high when idle      |

### CRSF UART

| Role | TX device            | RX device              |
|------|----------------------|------------------------|
| TX   | PC / flight controller | ESP32 GPIO 0 (CRSF TX) |
| RX   | ESP32 GPIO 1 (CRSF RX) | PC / flight controller |

- **TX build**: PC sends CRSF over UART -> ESP32 GPIO 1 (RX) -> RF out.
- **RX build**: RF in -> ESP32 GPIO 0 (TX) -> flight controller UART RX.

## ESP32-C3 available GPIOs

ESP32-C3 supports GPIOs: 0, 1, 4, 5, 6, 7, 8, 9, 10, 18, 19, 20, 21.  
This pinout uses only: **0, 1, 4, 5, 6, 7, 10, 18**.

To change pins, edit `src/config.h`.
