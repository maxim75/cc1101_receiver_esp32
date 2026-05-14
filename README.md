# CC1101 + nRF24L01 Dual RF Receiver (ESP32-S3)

ESP32-S3 firmware that receives RF packets from two radios simultaneously and displays decoded data on an SH1107 128×128 OLED and serial monitor.

- **CC1101** — 433.92 MHz OOK packets from an ATtiny3226 transmitter (ELECHOUSE SmartRC-compatible protocol)
- **nRF24L01** — 2.4 GHz packets at 250 kbps, PA MAX

## Hardware

**Board:** ESP32-S3-DevKitC-1 (16 MB flash, PSRAM, native USB CDC)

### Wiring

| Peripheral | Signal | GPIO |
|------------|--------|------|
| CC1101     | SCK    | 12 (FSPI/SPI2) |
|            | MOSI   | 11 |
|            | MISO   | 13 |
|            | CSN    | 10 |
|            | GDO0   | 2 (interrupt) |
| nRF24L01   | SCK    | 14 (HSPI/SPI3) |
|            | MOSI   | 1 |
|            | MISO   | 21 |
|            | CSN    | 6 |
|            | CE     | 5 |
|            | IRQ    | 4 (wired, not used) |
| SH1107     | SDA    | 8 |
|            | SCL    | 9 |

All peripherals: VCC → 3.3 V, GND → GND

## Build & Flash

Requires [PlatformIO](https://platformio.org/).

```bash
# Build only
pio run

# Flash
pio run --target upload

# Flash and open serial monitor
pio run --target upload && pio device monitor

# Serial monitor only (115200 baud)
pio device monitor
```

## Dependencies

Managed automatically by PlatformIO:

| Library | Purpose |
|---------|---------|
| `jgromes/RadioLib` | CC1101 driver |
| `nrf24/RF24` | nRF24L01 driver |
| `olikraus/U8g2` | SH1107 OLED driver |
| `bblanchon/ArduinoJson` | JSON (available, not yet used) |

## How It Works

All firmware lives in `src/main.cpp`.

**CC1101** — interrupt-driven: GDO0 RISING edge sets a flag; `loop()` reads the packet with RadioLib, re-arms `startReceive()`, then calls `getRSSI()`.

**nRF24L01** — polled: `loop()` calls `nrf24.available()` each iteration; no re-arm needed.

Both radios share the same `PacketInfo` struct (`source`, hex, ASCII, RSSI, count). After each packet the OLED and serial monitor are updated; the OLED header shows which radio sourced the last packet.

**SPI:** CC1101 on FSPI (`SPI`), nRF24 on HSPI (`hspi`) — separate buses, no CS conflicts.

## Serial Output

Each received packet is logged at 115200 baud:

```
[CC1101] #1 | RSSI: -72 dBm | HEX: D3 91 04 AB CD | ASCII: ...
[nRF24]  #2 | HEX: 48 65 6C 6C 6F     | ASCII: Hello
```
