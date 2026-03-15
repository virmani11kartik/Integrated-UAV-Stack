# ELRS TX Pinout - ESP32-WROOM + E28-2G4T12S

RX is an off-the-shelf ELRS receiver.

## Pin assignment table

| GPIO | Function   | Direction | Notes                              |
|------|------------|-----------|------------------------------------|
| 0    | CRSF TX    | Output    | BOOT strapping — do not hold low at boot |
| 1    | CRSF RX    | Input     | U0TXD — shared with serial monitor |
| 4    | E28 TX     | Output    | ESP32 -> E28 RXD                   |
| 5    | E28 RX     | Input     | E28 TXD -> ESP32                   |
| 25   | E28 M0     | Output    | Mode select                        |
| 26   | E28 M1     | Output    | Mode select                        |
| 27   | E28 M2     | Output    | Mode select                        |
| 18   | E28 AUX    | Input     | Busy/status, high = idle           |

## Wiring reference

### E28-2G4T12S module -> ESP32-WROOM

| E28 pin | ESP32-WROOM | Notes                            |
|---------|-------------|----------------------------------|
| VCC     | 3.3V        | 3.3V supply                      |
| GND     | GND         | Common ground                    |
| TXD     | GPIO 5      | E28 TX -> ESP32 RX               |
| RXD     | GPIO 4      | ESP32 TX -> E28 RX               |
| M0      | GPIO 25     | Mode select                      |
| M1      | GPIO 26     | Mode select                      |
| M2      | GPIO 27     | Mode select                      |
| AUX     | GPIO 18     | Busy/status, high when idle      |

### CRSF UART

| Role | TX device              | RX device                |
|------|------------------------|--------------------------|
| TX   | PC / flight controller | ESP32 GPIO 0 (CRSF TX)  |
| RX   | ESP32 GPIO 1 (CRSF RX) | PC / flight controller  |

- **TX build**: PC sends CRSF over UART -> ESP32 GPIO 1 (RX) -> RF out via E28.

## ESP32-WROOM GPIO notes

**Usable GPIOs**: 0, 1, 2, 3, 4, 5, 12-19, 21-23, 25-27, 32-33
**Input-only** (no output): 34, 35, 36, 39
**Flash pins** (do not use): 6, 7, 8, 9, 10, 11
**Strapping pins**: 0 (BOOT), 2, 5, 12 (VDD_FLASH), 15 (LOG)

This pinout uses: **0, 1, 4, 5, 18, 25, 26, 27** — all support GPIO output.

To change pins, edit `src/config.h`.
