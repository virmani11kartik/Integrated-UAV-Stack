# ELRS - ExpressLRS-style link for UAV-Controller

ESP32-WROOM **Transmitter (TX)** using an ExpressLRS-style over-the-air protocol. Pairs with any off-the-shelf ELRS 2.4 GHz receiver. Intended for autonomous flight: computer sends CRSF over UART to TX; RX outputs CRSF to the flight controller.

## Hardware

- **MCU**: ESP32-WROOM (e.g. esp32dev).
- **Radio**: Ebyte **E28-2G4T12S** UART module (M0/M1/M2/AUX + TX/RX).
- **RX**: Any off-the-shelf ELRS 2.4 GHz receiver.
- **UART**: CRSF @ 420000 baud (from computer to TX). Radio UART is separate.

Pin definitions are in `src/config.h`. See **[PINOUT.md](PINOUT.md)** for full wiring.

GPIOs used: **0, 1, 4, 5, 18, 25, 26, 27** — all support input and output on ESP32-WROOM.

| Signal      | GPIO |
|-------------|------|
| E28 TX      | 4    |
| E28 RX      | 5    |
| E28 M0      | 25   |
| E28 M1      | 26   |
| E28 M2      | 27   |
| E28 AUX     | 18   |
| CRSF TX     | 0    |
| CRSF RX     | 1    |

Adjust in `config.h` if your wiring differs. Avoid holding GPIO 0 low at boot (enters bootloader).
GPIO 6-11 are flash pins on ESP32-WROOM — do not use.

## Bind phrase

TX and RX must use the **same bind phrase** (and thus same UID) so FHSS and CRC match. Set in `src/config.h`:

```c
#define ELRS_BIND_PHRASE "elrsuav"
```

## Build and upload

```bash
cd ELRS

# Build
pio run -e elrs_tx

# Upload to ESP32-WROOM
pio run -e elrs_tx -t upload

# Monitor serial output
pio device monitor -e elrs_tx
```

## Setup and testing

### 1. Wire the E28 module to the ESP32-WROOM

Follow the wiring table above or [PINOUT.md](PINOUT.md). Connect:
- E28 **TXD** → ESP32 GPIO 5, E28 **RXD** → ESP32 GPIO 4
- E28 **M0** → GPIO 25, **M1** → GPIO 26, **M2** → GPIO 27
- E28 **AUX** → GPIO 18
- E28 **VCC** → 3.3V, **GND** → GND

### 2. Configure the bind phrase

In `src/config.h`, set the same bind phrase on the TX and your ELRS receiver:

```c
#define ELRS_BIND_PHRASE "elrsuav"
```

Or use classic binding (`ELRS_CLASSIC_BIND 1`) where the TX broadcasts its UID and the receiver auto-binds.

### 3. Build and flash

```bash
cd ELRS
pio run -e elrs_tx -t upload
```

### 4. Connect CRSF UART

Connect your computer (or flight software) UART to the ESP32 CRSF pins:
- Computer TX → ESP32 GPIO 1 (CRSF RX)
- Computer RX ← ESP32 GPIO 0 (CRSF TX)
- Baud rate: **420000**

### 5. Power on and test

1. Power the ESP32-WROOM + E28 module.
2. Power your ELRS receiver (on the drone / FC).
3. The TX will begin sending ELRS packets over the air at 500 Hz.
4. Send CRSF RC channel frames from your computer over UART — they get forwarded over RF to the receiver.
5. The ELRS receiver outputs CRSF to the flight controller.

### 6. Verify link

- **AUX pin (GPIO 18)**: HIGH = E28 idle/ready, LOW = busy transmitting. If it stays LOW, check wiring or E28 config.
- **Serial monitor**: If `ELRS_LOG_PORT` is not `-1` in config.h, you can monitor debug output at 115200 baud.
- **Receiver LED**: Most off-the-shelf ELRS receivers have a status LED — solid = linked, blinking = searching.

## Protocol summary

- **OTA**: 8-byte packets (OTA4-style): RCDATA (4 channels + switches) or SYNC (timing/FHSS). CRC and nonce for validation.
- **FHSS**: Sequence derived from bind phrase; with E28 this is a fixed channel (set via `E28_CHANNEL`) so hopping is a no-op.
- **CRSF**: Same frame format as ESPNOW_TX / UAV-Controller CRSF (16x11-bit channels, DVB-S2 CRC).

## Rate and interval

Default: 500 Hz packet rate (`ELRS_PACKET_INTERVAL_US = 2000`), hop every 2 packets (`ELRS_FHSS_HOP_INTERVAL`). Edit `src/config.h` to change.

## Notes

- This is a minimal ELRS-style stack for UAV-Controller. It is not a full ExpressLRS fork.
- The E28 module runs on a fixed channel configured via UART parameters. Update `E28_*` values in `src/config.h` to match your module settings.
