# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 firmware that receives OOK RF packets from an ATtiny3226 transmitter (ELECHOUSE SmartRC-compatible protocol) and displays decoded packet data (hex, ASCII, RSSI) on an SH1107 128x128 OLED and serial monitor.

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
| CC1101     | SCK    | 12 (FSPI/SPI2) |
|            | MOSI   | 11   |
|            | MISO   | 13   |
|            | CSN    | 10   |
|            | GDO0   | 2 (RISING interrupt) |
| nRF24L01   | SCK    | 14 (HSPI/SPI3) |
|            | MOSI   | 1    |
|            | MISO   | 21   |
|            | CSN    | 6    |
|            | CE     | 5    |
|            | IRQ    | 4 (FALLING interrupt) |
| SH1107     | SDA    | 8    |
|            | SCL    | 9    |

## Architecture

All firmware lives in `src/main.cpp`. The code is organized into namespaces and a single struct:

- **`Pin::`** — GPIO assignments; CC1101 on FSPI (SPI2, GPIO 11/12/13), nRF24 on HSPI (SPI3, GPIO 1/14/21), each radio has its own CS
- **`Radio::`** — CC1101 RF parameters (433.92 MHz, 4.8 kbps, OOK, sync word `0xD391`)
- **`Nrf::`** — nRF24L01 pipe address (must match transmitter); RF config is 250 kbps, PA MAX, set at runtime via RF24 APIs
- **`Display::`** — OLED layout constants (SH1107 128x128)
- **`PacketInfo`** — last received packet: `source[]`, hex, ASCII, RSSI, count, `hasRssi` flag

**Data flow:**
1. CC1101 GDO0 (RISING) → `onCc1101Packet()` ISR sets `cc1101Ready`; nRF24 is polled via `nrf24.available()` in `loop()`
2. `loop()` checks `cc1101Ready`, reads with `readData()`, re-arms `startReceive()`; polls nRF24 with `available()` + `read()`, no re-arm needed
3. `fillPacket()` populates `lastPacket` (hex truncated to 7 bytes for OLED, source label, RSSI if available)
4. `displayPacket()` and `logPacketToSerial()` render the result; the OLED header shows which radio received the last packet

**SPI buses:** CC1101 uses `SPI` (FSPI/SPI2, `SPI.begin(SCK, MISO, MOSI)`); nRF24L01 uses `hspi` (HSPI/SPI3, `hspi.begin(...)`) passed to `RF24::begin(&hspi)`. The nRF24 IRQ pin is wired but not used — the firmware polls `available()` instead.

**Key library APIs:**
- RadioLib `CC1101` — `setGdo0Action()`, `startReceive()`, `getPacketLength()`, `readData()`, `getRSSI()`
- RF24 `nRF24L01` — `begin(&spi)`, `setDataRate()`, `openReadingPipe()`, `setPALevel()`, `startListening()`, `available()`, `getDynamicPayloadSize()`, `getPayloadSize()`, `read()`
- U8g2 full-framebuffer mode (`_F_`) — `clearBuffer()` / `sendBuffer()` pattern for flicker-free updates

## Dependencies (managed by PlatformIO)

- `jgromes/RadioLib` — CC1101 driver
- `nrf24/RF24` — nRF24L01 driver
- `olikraus/U8g2` — SH1107 OLED driver
- `bblanchon/ArduinoJson@^6.21.0` — available but not yet used in main flow
