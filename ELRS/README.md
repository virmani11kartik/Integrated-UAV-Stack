# ELRS - ExpressLRS-style link for UAV-Controller

ESP32-C3 **Transmitter (TX)** and **Receiver (RX)** using an ExpressLRS-style over-the-air protocol. Intended for autonomous flight: computer sends CRSF over UART to TX; RX outputs CRSF to the flight controller.

## Hardware

- **MCU**: ESP32-C3 (e.g. esp32-c3-devkitm-1).
- **Radio**: Ebyte **E28-2G4T12S** UART module (M0/M1/M2/AUX + TX/RX).
- **UART**: CRSF @ 420000 baud (TX: from computer; RX: to flight controller). Radio UART is separate.

Pin definitions are in `src/config.h`. See **[PINOUT.md](PINOUT.md)** for a wiring diagram.

Only these ESP32-C3 GPIOs are used (default): **0, 1, 4, 5, 6, 7, 10, 18**.

| Signal      | GPIO |
|-------------|------|
| E28 TX      | 4    |
| E28 RX      | 5    |
| E28 M0      | 6    |
| E28 M1      | 7    |
| E28 M2      | 10   |
| E28 AUX     | 18   |
| CRSF TX     | 0    |
| CRSF RX     | 1    |

Adjust in `config.h` if your wiring differs. Avoid holding GPIO 0 low at boot (enters bootloader).

## Bind phrase

TX and RX must use the **same bind phrase** (and thus same UID) so FHSS and CRC match. Set in `src/config.h`:

```c
#define ELRS_BIND_PHRASE "elrsuav"
```

## Build and upload

```bash
cd ELRS

# Transmitter (computer -> RF)
pio run -e elrs_tx
pio run -e elrs_tx -t upload

# Receiver (RF -> flight controller)
pio run -e elrs_rx
pio run -e elrs_rx -t upload
```

## Usage

1. **TX**: Connect computer UART (CRSF) to CRSF TX/RX pins. Send CRSF RC channel frames; TX sends them over the air at 500 Hz (configurable in `config.h`).
2. **RX**: Power on after TX. After sync, RX outputs CRSF RC to the FC UART. Connect FC serial to CRSF TX/RX at 420000 baud.

## Protocol summary

- **OTA**: 8-byte packets (OTA4-style): RCDATA (4 channels + switches) or SYNC (timing/FHSS). CRC and nonce for validation.
- **FHSS**: Sequence derived from bind phrase; with E28 this is a fixed channel (set via `E28_CHANNEL`) so hopping is a no-op.
- **CRSF**: Same frame format as ESPNOW_TX / UAV-Controller CRSF (16x11-bit channels, DVB-S2 CRC).

## Rate and interval

Default: 500 Hz packet rate (`ELRS_PACKET_INTERVAL_US = 2000`), hop every 2 packets (`ELRS_FHSS_HOP_INTERVAL`). Edit `src/config.h` to change.

## Notes

- This is a minimal ELRS-style stack for UAV-Controller. It is not a full ExpressLRS fork.
- The E28 module runs on a fixed channel configured via UART parameters. Update `E28_*` values in `src/config.h` to match your module settings.
