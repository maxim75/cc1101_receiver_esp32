/**
 * @file    main.cpp
 * @brief   ESP32-S3 + CC1101 RF receiver with SH1106 OLED status display.
 *
 * Receives OOK packets transmitted by an ATtiny3226 (ELECHOUSE SmartRC-compatible).
 * Decoded packet details (hex, ASCII, RSSI) are shown on the OLED and serial port.
 *
 * Libraries
 *   RadioLib  — jgromes/RadioLib
 *   U8g2      — olikraus/U8g2
 *
 * Wiring
 *   Peripheral  Signal  ESP32-S3 GPIO
 *   ─────────────────────────────────
 *   CC1101      SCK     12
 *               MOSI    11
 *               MISO    13
 *               CSN     10
 *               GDO0     2   (packet interrupt)
 *   SH1106      SDA      8
 *               SCL      9
 *   Both        VCC     3.3V
 *               GND     GND
 */

#include <RadioLib.h>
#include <SPI.h>
#include <U8g2lib.h>

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------

namespace Pin {
    // CC1101 (SPI using ESP32-S3 standard FSPI pins)
    constexpr int CC_SCK  = 12;
    constexpr int CC_MISO = 13;
    constexpr int CC_MOSI = 11;
    constexpr int CC_CS   = 10;
    constexpr int CC_GDO0 =  2;

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

namespace Display {
    constexpr uint8_t WIDTH         = 128;
    constexpr uint8_t HEIGHT        =  64;
    constexpr uint8_t LINE_HEIGHT   =  14;
    constexpr int     HEX_MAX_BYTES =   7;    // bytes shown on OLED before "..."
}

// ---------------------------------------------------------------------------
// Peripherals
// ---------------------------------------------------------------------------

CC1101 radio = new Module(Pin::CC_CS, Pin::CC_GDO0, RADIOLIB_NC, RADIOLIB_NC);

// Full-framebuffer SH1106, hardware I2C, explicit SCL/SDA pins
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(
    U8G2_R0, U8X8_PIN_NONE, Pin::OLED_SCL, Pin::OLED_SDA);

// ---------------------------------------------------------------------------
// Application state
// ---------------------------------------------------------------------------

struct PacketInfo {
    char     hex[48]   = {};
    char     ascii[22] = {};
    float    rssi      = 0.0f;
    uint32_t count     = 0;
    bool     valid     = false;
};

static PacketInfo lastPacket;
static volatile bool packetReady = false;

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------

void IRAM_ATTR onPacketReceived() {
    packetReady = true;
}

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

    // Header: "CC1101 RX" on left, packet counter on right
    display.drawStr(0, 10, "CC1101 RX");
    if (pkt.count > 0) {
        char counter[12];
        snprintf(counter, sizeof(counter), "#%lu", pkt.count);
        display.drawStr(Display::WIDTH - display.getStrWidth(counter), 10, counter);
    }
    display.drawHLine(0, 12, Display::WIDTH);

    if (!pkt.valid) {
        display.drawStr(0, 32, "Listening...");
    } else {
        char rssiLine[20];
        snprintf(rssiLine, sizeof(rssiLine), "RSSI: %.0f dBm", pkt.rssi);
        display.drawStr(0, 26, rssiLine);
        display.drawStr(0, 40, pkt.hex);

        char asciiLine[24];
        snprintf(asciiLine, sizeof(asciiLine), "\"%s\"", pkt.ascii);
        display.drawStr(0, 54, asciiLine);
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

static void logPacketToSerial(const PacketInfo& pkt, const uint8_t* rawData, int len) {
    Serial.printf("── Packet %lu (%d bytes) ─────────────────\n", pkt.count, len);
    Serial.print("  Hex:   ");
    for (int i = 0; i < len; i++) Serial.printf("%02X ", rawData[i]);
    Serial.println();
    Serial.printf("  ASCII: \"%s\"\n", pkt.ascii);
    Serial.printf("  RSSI:  %.1f dBm\n", pkt.rssi);
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
    displaySplash("CC1101 RX", "Initializing...");

    Serial.println("\n==============================");
    Serial.println("  ESP32-S3 CC1101 Receiver");
    Serial.println("==============================");

    SPI.begin(Pin::CC_SCK, Pin::CC_MISO, Pin::CC_MOSI, Pin::CC_CS);

    int state = radio.begin(
        Radio::FREQUENCY_MHZ,
        Radio::BITRATE_KBPS,
        Radio::DEVIATION_KHZ,
        Radio::RXBW_KHZ,
        Radio::POWER_DBM,
        Radio::PREAMBLE_BITS);

    if (state != RADIOLIB_ERR_NONE) {
        char errMsg[24];
        snprintf(errMsg, sizeof(errMsg), "Err code: %d", state);
        Serial.printf("[ERROR] CC1101 init failed: %d\n", state);
        displaySplash("CC1101 FAILED", errMsg);
        while (true) delay(1000);
    }

    radio.setOOK(true);
    radio.setSyncWord(Radio::SYNC_BYTE_1, Radio::SYNC_BYTE_2);
    radio.setCrcFiltering(true);
    radio.setGdo0Action(onPacketReceived, RISING);
    radio.startReceive();

    Serial.println("[OK] CC1101 ready");
    Serial.printf("  Frequency:  %.2f MHz\n",  Radio::FREQUENCY_MHZ);
    Serial.printf("  Data rate:  %.1f kbps\n", Radio::BITRATE_KBPS);
    Serial.printf("  Sync word:  0x%02X%02X\n", Radio::SYNC_BYTE_1, Radio::SYNC_BYTE_2);
    Serial.println("  Modulation: OOK | CRC: enabled");
    Serial.println("\n[LISTENING]\n");

    displayPacket(lastPacket);  // shows "Listening..."
}

void loop() {
    if (!packetReady) return;
    packetReady = false;

    const int len = radio.getPacketLength();
    if (len <= 0) {
        radio.startReceive();
        return;
    }

    uint8_t buf[64];
    const int state = radio.readData(buf, len);
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[WARN] readData error: %d\n", state);
        return;
    }

    lastPacket.count++;
    lastPacket.rssi  = radio.getRSSI();
    lastPacket.valid = true;
    buildHexString(buf, len, lastPacket.hex, sizeof(lastPacket.hex));
    buildAsciiString(buf, len, lastPacket.ascii, sizeof(lastPacket.ascii));

    displayPacket(lastPacket);
    logPacketToSerial(lastPacket, buf, len);
}
