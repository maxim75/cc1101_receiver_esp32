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

ESP32-S3 firmware that receives fixed-code 433 MHz remotes (EV1527/PT2262) via a CC1101 and 2.4 GHz packets via an nRF24L01, and displays decoded data (hex, remote code info or ASCII, RSSI) on an SH1107 128x128 OLED and serial monitor. Received events are published over Wi-Fi/MQTT for Home Assistant.

## Build & Flash Commands

First build requires `include/secrets.h` (gitignored): copy `include/secrets.example.h` and fill in Wi-Fi and MQTT credentials. The build fails with an `#error` if it is missing.

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
|            | GDO0   | 2 (async OOK data, CHANGE interrupt) |
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
- **`Radio::`** — CC1101 RF parameters (433.92 MHz, OOK async, 270 kHz RX BW) and `REPEAT_GAP_MS` for collapsing repeated remote frames into one press
- **`Nrf::`** — nRF24L01 pipe address (must match transmitter); RF config is 250 kbps, PA MAX, applied by `initNrf24()`; `HEALTH_CHECK_MS` sets the brownout-reset check interval
- **`Mqtt::`** — client ID `rfgateway`, topics, retry interval, buffer size
- **`Display::`** — OLED layout constants (SH1107 128x128, rotated `U8G2_R3`): header + 10-row log geometry
- **`Clock::`** — Sydney POSIX TZ string, NTP server, `VALID_AFTER` epoch used to detect an unsynced clock
- **`PacketInfo`** — last received packet for serial logging: `source[]`, ASCII, RSSI, count, `hasRssi` flag
- **`LogEntry` / `msgLog[]`** — rolling OLED message log, `[0]` = newest; each entry stores receive time (`0` if clock unsynced) and display text

**Data flow:**
1. CC1101 runs in async direct mode (`receiveDirectAsync()`): GDO0 outputs the raw demodulated OOK signal, and rc-switch's own CHANGE interrupt on GDO0 decodes pulse timings; nRF24 is polled via `nrf24.available()` in `loop()`
2. `loop()` polls `rcSwitch.available()`; a code identical to the previous one within `REPEAT_GAP_MS` is treated as the same press and skipped. nRF24 is read with `available()` + `read()`, no re-arm needed
3. `fillPacket()` populates `lastPacket` (source label, ASCII, RSSI if available) for serial logging; `addMessage()` pushes `RF <code>` or the nRF24 text (no prefix, cut at first NUL to hide padding) onto `msgLog` and marks the screen dirty
4. `logRemoteToSerial()` / `logPacketToSerial()` print full details (code, bits, protocol, pulse, RSSI / hex, ASCII)
5. `publishRemote()` / `publishNrf()` send the event to MQTT (dropped silently while disconnected)
6. `drawScreen()` runs at the end of `loop()` when the screen is dirty or the wall-clock second changes

**OLED layout:** header line 1 = IP address (or `Wi-Fi connecting...`); line 2 = Wi-Fi and MQTT dots (filled = up, hollow = down) plus the `HH:MM:SS` clock; below a rule, up to 10 log rows `HH:MM:SS <content>` (remotes prefixed `RF`, nRF24 text unprefixed) (5x8 font, content cut to 16 chars with `..`), newest row inverted. Times show `--:--:--` until NTP sync (`configTzTime()` with the Sydney TZ in `setup()`). The SEEED SH1107 driver runs I2C at 200 kHz, so a full redraw takes ~100 ms.

**MQTT / Home Assistant:** Wi-Fi starts in `setup()` without blocking; `mqttMaintain()` at the top of `loop()` logs Wi-Fi transitions, runs `mqtt.loop()`, and retries the broker every 5 s. A connect attempt to an unreachable broker can block `loop()` for a few seconds.
| Topic | Payload | Retained |
|---|---|---|
| `rfgateway/remote` | `{"code":…,"bits":…,"protocol":…,"pulse":…,"rssi":…}` | no |
| `nrf/message` | raw packet text up to first NUL (same format as the legacy `nrf_receiver` firmware) | no |
| `rfgateway/status` | `online` / `offline` (LWT) | yes |
| `homeassistant/sensor/rfgateway/*/config` | HA discovery for "Last remote code" and "Last nRF24 packet" sensors, published on every connect | yes |

HA automations should use an MQTT trigger on the topic, not the discovery sensors' state (repeated identical codes do not change state).

**nRF24 brownout recovery:** on this hardware the nRF24 module resets itself to power-on defaults (CONFIG `0x08`, 2 Mbps, RX payload width 0) under supply dips — observed every 1–2 packets while traffic flows — and then silently stops receiving. `loop()` checks `getDataRate()` every `Nrf::HEALTH_CHECK_MS`; anything other than 250 kbps means a reset, so `initNrf24()` is re-run and `[WARN] nRF24 reset detected, reinitialised` is logged. Packets arriving during the up-to-1 s gap are lost. The real fix is hardware: 10–100 µF + 100 nF across the module's VCC/GND.

**SPI buses:** CC1101 uses `SPI` (FSPI/SPI2, `SPI.begin(SCK, MISO, MOSI)`); nRF24L01 uses `hspi` (HSPI/SPI3, `hspi.begin(...)`) passed to `RF24::begin(&hspi)`. The nRF24 IRQ pin is wired but not used — the firmware polls `available()` instead.

**Key library APIs:**
- RadioLib `CC1101` — `setOOK()`, `receiveDirectAsync()`, `getRSSI()` (live RSSI register read in direct mode)
- rc-switch `RCSwitch` — `enableReceive()`, `available()`, `getReceivedValue()`, `getReceivedBitlength()`, `getReceivedProtocol()`, `getReceivedDelay()`, `resetAvailable()`
- RF24 `nRF24L01` — `begin(&spi)`, `setDataRate()`, `openReadingPipe()`, `setPALevel()`, `startListening()`, `available()`, `getDynamicPayloadSize()`, `getPayloadSize()`, `read()`
- PubSubClient — `setServer()`, `setBufferSize()`, `connect()` with LWT, `publish()`, `loop()`
- U8g2 full-framebuffer mode (`_F_`) — `clearBuffer()` / `sendBuffer()` pattern for flicker-free updates

## Dependencies (managed by PlatformIO)

- `jgromes/RadioLib` — CC1101 driver
- `sui77/rc-switch` — EV1527/PT2262 remote pulse decoder
- `nrf24/RF24` — nRF24L01 driver
- `olikraus/U8g2` — SH1107 OLED driver
- `knolleary/PubSubClient` — MQTT client
- `bblanchon/ArduinoJson@^6.21.0` — builds HA discovery payloads
