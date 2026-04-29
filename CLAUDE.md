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
| CC1101     | SCK    | 12   |
|            | MOSI   | 11   |
|            | MISO   | 13   |
|            | CSN    | 10   |
|            | GDO0   | 2 (packet interrupt) |
| SH1106     | SDA    | 8    |
|            | SCL    | 9    |

## Architecture

All firmware lives in `src/main.cpp`. The code is organized into namespaces and a single struct:

- **`Pin::`** — GPIO pin assignments (CC1101 SPI + OLED I2C)
- **`Radio::`** — RF parameters (433.92 MHz, 4.8 kbps, OOK, sync word `0xD391`)
- **`Display::`** — OLED layout constants
- **`PacketInfo`** — holds the last received packet (hex string, ASCII string, RSSI, count, valid flag)

**Data flow:**
1. CC1101 GDO0 pin triggers `onPacketReceived()` ISR (IRAM, sets `packetReady` flag)
2. `loop()` polls `packetReady`, reads packet via RadioLib `readData()`, immediately re-arms `startReceive()`
3. Packet is formatted into `lastPacket` (hex truncated to 7 bytes for OLED, full hex to serial)
4. `displayPacket()` and `logPacketToSerial()` render the result

**Key library APIs:**
- RadioLib `CC1101` class — `begin()`, `setOOK()`, `setSyncWord()`, `setCrcFiltering()`, `setGdo0Action()`, `startReceive()`, `getPacketLength()`, `readData()`
- U8g2 full-framebuffer mode (`_F_`) — `clearBuffer()` / `sendBuffer()` pattern for flicker-free updates

## Dependencies (managed by PlatformIO)

- `jgromes/RadioLib` — CC1101 driver
- `olikraus/U8g2` — SH1106 OLED driver
- `bblanchon/ArduinoJson@^6.21.0` — available but not yet used in main flow
