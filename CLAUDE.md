# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 firmware that receives OOK RF packets from an ATtiny3226 transmitter (ELECHOUSE SmartRC-compatible protocol) and displays decoded packet data (hex, ASCII, RSSI) on an SH1106 OLED and serial monitor.

## Build & Flash Commands

```bash
# Build
pio run

# Build, flash, and open serial monitor
pio run --target upload && pio device monitor

# Flash only
pio run --target upload

# Serial monitor only (115200 baud, /dev/cu.usbmodem1301)
pio device monitor

# Clean build artifacts
pio run --target clean
```

## Hardware

**Target board:** ESP32-S3-DevKitC-1 (16MB flash, PSRAM enabled, native USB CDC)

**Wiring:**
| Peripheral | Signal | GPIO |
|------------|--------|------|
| (shared)   | SCK    | 12   |
|            | MOSI   | 11   |
|            | MISO   | 13   |
| CC1101     | CSN    | 10   |
|            | GDO0   | 2 (RISING interrupt) |
| nRF24L01   | CSN    | 6    |
|            | CE     | 5    |
|            | IRQ    | 4 (FALLING interrupt) |
| SH1106     | SDA    | 8    |
|            | SCL    | 9    |

## Architecture

All firmware lives in `src/main.cpp`. The code is organized into namespaces and a single struct:

- **`Pin::`** — GPIO assignments; SCK/MOSI/MISO are shared bus constants, each radio has its own CS
- **`Radio::`** — CC1101 RF parameters (433.92 MHz, 4.8 kbps, OOK, sync word `0xD391`)
- **`Nrf::`** — nRF24L01 parameters (2400 MHz ch 0, 1 Mbps, −12 dBm)
- **`Display::`** — OLED layout constants
- **`PacketInfo`** — last received packet: `source[]`, hex, ASCII, RSSI, count, `hasRssi` flag

**Data flow:**
1. CC1101 GDO0 (RISING) → `onCc1101Packet()` ISR sets `cc1101Ready`; nRF24 IRQ (FALLING, via `attachInterrupt`) → `onNrf24Packet()` sets `nrf24Ready`
2. `loop()` checks each flag independently, reads the packet with `readData()`, immediately re-arms `startReceive()`
3. `fillPacket()` populates `lastPacket` (hex truncated to 7 bytes for OLED, source label, RSSI if available)
4. `displayPacket()` and `logPacketToSerial()` render the result; the OLED header shows which radio received the last packet

**SPI sharing:** `SPI.begin(SCK, MISO, MOSI)` is called once; RadioLib manages each module's CS pin independently. nRF24 uses `attachInterrupt` rather than a RadioLib-specific callback to avoid IRQ-clear ordering issues.

**Key library APIs:**
- RadioLib `CC1101` — `setGdo0Action()`, `startReceive()`, `getPacketLength()`, `readData()`, `getRSSI()`
- RadioLib `nRF24` — `begin(freq, dataRate, power)`, `startReceive()`, `getPacketLength()`, `readData()`
- U8g2 full-framebuffer mode (`_F_`) — `clearBuffer()` / `sendBuffer()` pattern for flicker-free updates

## Dependencies (managed by PlatformIO)

- `jgromes/RadioLib` — CC1101 and nRF24L01 drivers
- `olikraus/U8g2` — SH1106 OLED driver
- `bblanchon/ArduinoJson@^6.21.0` — available but not yet used in main flow
