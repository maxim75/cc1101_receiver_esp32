/**
 * @file    main.cpp
 * @brief   ESP32-S3 + CC1101 + nRF24L01 dual RF receiver with SH1106 OLED display.
 *
 * CC1101  — receives OOK packets from an ATtiny3226 (ELECHOUSE SmartRC-compatible).
 * nRF24L01— receives 2.4 GHz packets on the same FSPI bus.
 * Decoded packet details (hex, ASCII, RSSI where available) are shown on the OLED
 * and serial port.
 *
 * Libraries
 *   RadioLib  — jgromes/RadioLib
 *   U8g2      — olikraus/U8g2
 *
 * Wiring
 *   Peripheral  Signal  ESP32-S3 GPIO
 *   ─────────────────────────────────
 *   (shared)    SCK     12  ┐
 *               MOSI    11  │ FSPI bus
 *               MISO    13  ┘
 *   CC1101      CSN     10
 *               GDO0     2   (packet interrupt, RISING)
 *   nRF24L01    CSN      6
 *               CE       5
 *               IRQ      4   (packet interrupt, FALLING)
 *   SH1106      SDA      8
 *               SCL      9
 *   All         VCC     3.3V
 *               GND     GND
 */

#include <RadioLib.h>
#include <SPI.h>
#include <U8g2lib.h>

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------

namespace Pin {
    // Shared FSPI bus
    constexpr int SCK  = 12;
    constexpr int MISO = 13;
    constexpr int MOSI = 11;

    // CC1101
    constexpr int CC_CS   = 10;
    constexpr int CC_GDO0 =  2;

    // nRF24L01
    constexpr int NRF_CS  =  6;
    constexpr int NRF_CE  =  5;
    constexpr int NRF_IRQ =  4;

    // SH1106 (I2C)
    constexpr int OLED_SDA = 8;
    constexpr int OLED_SCL = 9;
}

namespace Radio {
    constexpr float   FREQUENCY_MHZ  = 433.92f;
    constexpr float   BITRATE_KBPS   =   4.8f;
    constexpr float   DEVIATION_KHZ  =   5.157f;
    constexpr float   RXBW_KHZ       = 203.0f;
    constexpr int     POWER_DBM      =  10;
    constexpr int     PREAMBLE_BITS  =  16;    // 2 bytes, CC1101 minimum
    constexpr uint8_t SYNC_BYTE_1    = 0xD3;
    constexpr uint8_t SYNC_BYTE_2    = 0x91;
}

namespace Nrf {
    constexpr int16_t FREQUENCY_MHZ = 2476;    // channel 0; increment by 1 per channel
    constexpr int16_t DATA_RATE_KBPS = 250;   // 250, 1000, or 2000
    constexpr int8_t  POWER_DBM      =  -12;
}

namespace Display {
    constexpr uint8_t WIDTH         = 128;
    constexpr uint8_t HEIGHT        =  64;
    constexpr int     HEX_MAX_BYTES =   7;    // bytes shown on OLED before "..."
}

// ---------------------------------------------------------------------------
// Peripherals
// ---------------------------------------------------------------------------

CC1101 cc1101 = new Module(Pin::CC_CS, Pin::CC_GDO0, RADIOLIB_NC, RADIOLIB_NC);
nRF24  nrf24  = new Module(Pin::NRF_CS, Pin::NRF_IRQ, RADIOLIB_NC, Pin::NRF_CE);

// Full-framebuffer SH1106, hardware I2C, explicit SCL/SDA pins
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(
    U8G2_R0, U8X8_PIN_NONE, Pin::OLED_SCL, Pin::OLED_SDA);

// ---------------------------------------------------------------------------
// Application state
// ---------------------------------------------------------------------------

struct PacketInfo {
    char     source[8]  = {};
    char     hex[48]    = {};
    char     ascii[22]  = {};
    float    rssi       = 0.0f;
    uint32_t count      = 0;
    bool     valid      = false;
    bool     hasRssi    = false;
};

static PacketInfo lastPacket;
static volatile bool cc1101Ready = false;
static volatile bool nrf24Ready  = false;

// ---------------------------------------------------------------------------
// ISRs
// ---------------------------------------------------------------------------

void IRAM_ATTR onCc1101Packet() { cc1101Ready = true; }
void IRAM_ATTR onNrf24Packet()  { nrf24Ready  = true; }

// ---------------------------------------------------------------------------
// Display helpers
// ---------------------------------------------------------------------------

static void displaySplash(const char* line1, const char* line2 = nullptr) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tr);
    display.drawStr(0, 20, line1);
    if (line2) display.drawStr(0, 36, line2);
    display.sendBuffer();
}

static void displayPacket(const PacketInfo& pkt) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tr);

    // Header: source on left, packet counter on right
    display.drawStr(0, 10, pkt.valid ? pkt.source : "Dual RX");
    if (pkt.count > 0) {
        char counter[12];
        snprintf(counter, sizeof(counter), "#%lu", pkt.count);
        display.drawStr(Display::WIDTH - display.getStrWidth(counter), 10, counter);
    }
    display.drawHLine(0, 12, Display::WIDTH);

    if (!pkt.valid) {
        display.drawStr(0, 32, "Listening...");
    } else {
        uint8_t y = 26;
        if (pkt.hasRssi) {
            char rssiLine[20];
            snprintf(rssiLine, sizeof(rssiLine), "RSSI: %.0f dBm", pkt.rssi);
            display.drawStr(0, y, rssiLine);
            y += 14;
        }
        display.drawStr(0, y, pkt.hex);   y += 14;

        char asciiLine[24];
        snprintf(asciiLine, sizeof(asciiLine), "\"%s\"", pkt.ascii);
        display.drawStr(0, y, asciiLine);
    }

    display.sendBuffer();
}

// ---------------------------------------------------------------------------
// Radio helpers
// ---------------------------------------------------------------------------

static void buildHexString(const uint8_t* data, int len, char* out, size_t outSize) {
    out[0] = '\0';
    int shown = min(len, Display::HEX_MAX_BYTES);
    for (int i = 0; i < shown; i++) {
        char tmp[4];
        snprintf(tmp, sizeof(tmp), "%02X ", data[i]);
        strncat(out, tmp, outSize - strlen(out) - 1);
    }
    if (len > Display::HEX_MAX_BYTES) {
        strncat(out, "...", outSize - strlen(out) - 1);
    }
}

static void buildAsciiString(const uint8_t* data, int len, char* out, size_t outSize) {
    int maxChars = min(len, (int)(outSize - 1));
    for (int i = 0; i < maxChars; i++) {
        out[i] = isprint(data[i]) ? (char)data[i] : '.';
    }
    out[maxChars] = '\0';
}

static void fillPacket(PacketInfo& pkt, const char* source,
                       const uint8_t* buf, int len, float rssi, bool hasRssi) {
    pkt.count++;
    pkt.valid   = true;
    pkt.rssi    = rssi;
    pkt.hasRssi = hasRssi;
    strncpy(pkt.source, source, sizeof(pkt.source) - 1);
    pkt.source[sizeof(pkt.source) - 1] = '\0';
    buildHexString(buf, len, pkt.hex, sizeof(pkt.hex));
    buildAsciiString(buf, len, pkt.ascii, sizeof(pkt.ascii));
}

static void logPacketToSerial(const PacketInfo& pkt, const uint8_t* rawData, int len) {
    Serial.printf("── [%s] Packet %lu (%d bytes) ──────────\n", pkt.source, pkt.count, len);
    Serial.print("  Hex:   ");
    for (int i = 0; i < len; i++) Serial.printf("%02X ", rawData[i]);
    Serial.println();
    Serial.printf("  ASCII: \"%s\"\n", pkt.ascii);
    if (pkt.hasRssi) Serial.printf("  RSSI:  %.1f dBm\n", pkt.rssi);
    Serial.println("─────────────────────────────────────────\n");
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    // Wait up to 3 s for USB CDC host (ESP32-S3 native USB)
    for (uint32_t t = millis(); !Serial && (millis() - t < 3000); ) delay(10);

    // OLED up first so errors are visible on screen
    display.begin();
    displaySplash("Dual RX", "Initializing...");

    Serial.println("\n==============================");
    Serial.println("  ESP32-S3 Dual RF Receiver");
    Serial.println("==============================");

    // Single SPI.begin() — both radios share this bus via separate CS pins
    SPI.begin(Pin::SCK, Pin::MISO, Pin::MOSI);

    // --- CC1101 ---
    int state = cc1101.begin(
        Radio::FREQUENCY_MHZ,
        Radio::BITRATE_KBPS,
        Radio::DEVIATION_KHZ,
        Radio::RXBW_KHZ,
        Radio::POWER_DBM,
        Radio::PREAMBLE_BITS);

    if (state != RADIOLIB_ERR_NONE) {
        char errMsg[24];
        snprintf(errMsg, sizeof(errMsg), "CC1101 err: %d", state);
        Serial.printf("[ERROR] CC1101 init failed: %d\n", state);
        displaySplash("CC1101 FAILED", errMsg);
        while (true) delay(1000);
    }

    cc1101.setOOK(true);
    cc1101.setSyncWord(Radio::SYNC_BYTE_1, Radio::SYNC_BYTE_2);
    cc1101.setCrcFiltering(true);
    cc1101.setGdo0Action(onCc1101Packet, RISING);
    cc1101.startReceive();

    Serial.println("[OK] CC1101 ready");
    Serial.printf("  Frequency:  %.2f MHz\n",  Radio::FREQUENCY_MHZ);
    Serial.printf("  Data rate:  %.1f kbps\n", Radio::BITRATE_KBPS);
    Serial.printf("  Sync word:  0x%02X%02X\n", Radio::SYNC_BYTE_1, Radio::SYNC_BYTE_2);
    Serial.println("  Modulation: OOK | CRC: enabled");

    // --- nRF24L01 ---
    state = nrf24.begin(Nrf::FREQUENCY_MHZ, Nrf::DATA_RATE_KBPS, Nrf::POWER_DBM);
    if (state != RADIOLIB_ERR_NONE) {
        char errMsg[24];
        snprintf(errMsg, sizeof(errMsg), "nRF24 err: %d", state);
        Serial.printf("[ERROR] nRF24 init failed: %d\n", state);
        displaySplash("nRF24 FAILED", errMsg);
        while (true) delay(1000);
    }

    // IRQ is active-low; use attachInterrupt so RadioLib ISR internals don't interfere
    attachInterrupt(digitalPinToInterrupt(Pin::NRF_IRQ), onNrf24Packet, FALLING);
    nrf24.startReceive();

    Serial.println("[OK] nRF24 ready");
    Serial.printf("  Frequency:  %d MHz (ch %d)\n",
                  Nrf::FREQUENCY_MHZ, Nrf::FREQUENCY_MHZ - 2400);
    Serial.printf("  Data rate:  %d kbps\n", Nrf::DATA_RATE_KBPS);
    Serial.println("\n[LISTENING]\n");

    displayPacket(lastPacket);  // shows "Listening..."
}

void loop() {
    if (cc1101Ready) {
        cc1101Ready = false;

        const int len = cc1101.getPacketLength();
        if (len <= 0) { cc1101.startReceive(); return; }

        uint8_t buf[64];
        const int state = cc1101.readData(buf, len);
        cc1101.startReceive();

        if (state != RADIOLIB_ERR_NONE) {
            Serial.printf("[WARN] CC1101 readData error: %d\n", state);
            return;
        }

        fillPacket(lastPacket, "CC1101", buf, len, cc1101.getRSSI(), true);
        displayPacket(lastPacket);
        logPacketToSerial(lastPacket, buf, len);
    }

    if (nrf24Ready) {
        nrf24Ready = false;

        uint8_t buf[32];
        int len = nrf24.getPacketLength();
        if (len <= 0) len = sizeof(buf);    // fall back to max fixed payload

        const int state = nrf24.readData(buf, len);
        nrf24.startReceive();

        if (state != RADIOLIB_ERR_NONE) {
            Serial.printf("[WARN] nRF24 readData error: %d\n", state);
            return;
        }

        fillPacket(lastPacket, "nRF24", buf, len, 0.0f, false);
        displayPacket(lastPacket);
        logPacketToSerial(lastPacket, buf, len);
    }
}
