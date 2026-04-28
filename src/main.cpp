/**
 * ESP32-S3 + CC1101 Receiver with SH1106 OLED display
 * Library: RadioLib (jgromes/RadioLib), U8g2 (olikraus/U8g2)
 *
 * Matched to ATtiny3226 transmitter (ELECHOUSE SmartRC):
 *   OOK modulation, 4.8 kbps, sync 0xD391, CRC enabled
 *
 * ── Wiring ───────────────────────────────────────────────────────────────────
 *   CC1101    ESP32-S3
 *   VCC       3.3V
 *   GND       GND
 *   SCK       GPIO 40
 *   MOSI      GPIO 41
 *   MISO      GPIO 42
 *   CSN       GPIO 38
 *   GDO0      GPIO 2     packet interrupt
 *
 *   SH1106    ESP32-S3
 *   VCC       3.3V
 *   GND       GND
 *   SDA       GPIO 8
 *   SCL       GPIO 9
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <RadioLib.h>
#include <SPI.h>
#include <U8g2lib.h>

// ── CC1101 pins ──────────────────────────────────────────────────────────────
#define CC1101_SCK   40
#define CC1101_MISO  42
#define CC1101_MOSI  41
#define CC1101_CS    38
#define CC1101_GDO0   2

// ── SH1106 I2C pins ──────────────────────────────────────────────────────────
#define OLED_SDA      8
#define OLED_SCL      9

CC1101 radio = new Module(CC1101_CS, CC1101_GDO0, RADIOLIB_NC, RADIOLIB_NC);

// Hardware I2C with explicit SDA/SCL pins (reset=none, clock=SCL, data=SDA)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

volatile bool receivedFlag = false;

// ── Display state ────────────────────────────────────────────────────────────
static uint32_t packetCount = 0;
static char     dispHex[48]   = {};
static char     dispAscii[22] = {};
static float    dispRssi      = 0.0f;
static bool     hasPacket     = false;

void IRAM_ATTR onReceive() {
    receivedFlag = true;
}

// ── Draw current state to OLED ───────────────────────────────────────────────
void drawDisplay() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);

    // Header row: title + packet counter
    u8g2.drawStr(0, 10, "CC1101 RX");
    if (packetCount > 0) {
        char cnt[12];
        snprintf(cnt, sizeof(cnt), "#%lu", packetCount);
        u8g2.drawStr(128 - (int)u8g2.getStrWidth(cnt), 10, cnt);
    }
    u8g2.drawHLine(0, 12, 128);

    if (!hasPacket) {
        u8g2.drawStr(0, 32, "Listening...");
    } else {
        // Row 2: RSSI
        char rssiStr[20];
        snprintf(rssiStr, sizeof(rssiStr), "RSSI: %.0f dBm", dispRssi);
        u8g2.drawStr(0, 26, rssiStr);

        // Row 3: hex bytes (up to ~21 chars wide)
        u8g2.drawStr(0, 40, dispHex);

        // Row 4: ASCII payload
        char asciiLine[24];
        snprintf(asciiLine, sizeof(asciiLine), "\"%s\"", dispAscii);
        u8g2.drawStr(0, 54, asciiLine);
    }

    u8g2.sendBuffer();
}

void setup() {
    Serial.begin(115200);
    for (uint32_t t = millis(); !Serial && (millis() - t < 3000); ) delay(10);

    // Init OLED first so we can show status during radio init
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 20, "CC1101 RX");
    u8g2.drawStr(0, 36, "Initializing...");
    u8g2.sendBuffer();

    Serial.println("\n==============================");
    Serial.println("  ESP32 CC1101 Receiver");
    Serial.println("==============================");

    SPI.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);

    int state = radio.begin(433.92, 4.8, 5.157, 203.0, 10, 16);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] CC1101 init failed: %d\n", state);
        u8g2.clearBuffer();
        u8g2.drawStr(0, 20, "CC1101 ERROR");
        char errStr[20];
        snprintf(errStr, sizeof(errStr), "Code: %d", state);
        u8g2.drawStr(0, 36, errStr);
        u8g2.sendBuffer();
        while (true) delay(1000);
    }
    Serial.println("[OK] CC1101 detected");

    radio.setOOK(true);
    radio.setSyncWord(0xD3, 0x91);
    radio.setCrcFiltering(true);

    radio.setGdo0Action(onReceive, RISING);
    radio.startReceive();

    Serial.println("  Modulation: OOK");
    Serial.println("  Frequency:  433.92 MHz");
    Serial.println("  Data rate:  4.8 kbps");
    Serial.println("  Sync word:  0xD391");
    Serial.println("  CRC:        enabled");
    Serial.println("\n[LISTENING]\n");

    drawDisplay();
}

void loop() {
    if (!receivedFlag) return;
    receivedFlag = false;

    int len = radio.getPacketLength();
    if (len <= 0) {
        radio.startReceive();
        return;
    }

    uint8_t buf[64];
    int state = radio.readData(buf, len);
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[WARN] readData: %d\n", state);
        return;
    }

    packetCount++;
    dispRssi = radio.getRSSI();

    // Build hex string — up to 7 bytes shown, then "..."
    dispHex[0] = '\0';
    int hexBytes = (len < 7) ? len : 7;
    for (int i = 0; i < hexBytes; i++) {
        char tmp[4];
        snprintf(tmp, sizeof(tmp), "%02X ", buf[i]);
        strncat(dispHex, tmp, sizeof(dispHex) - strlen(dispHex) - 1);
    }
    if (len > 7) strncat(dispHex, "...", sizeof(dispHex) - strlen(dispHex) - 1);

    // Build ASCII string — up to 21 chars
    int asciiMax = (len < (int)(sizeof(dispAscii) - 1)) ? len : (int)(sizeof(dispAscii) - 1);
    for (int i = 0; i < asciiMax; i++)
        dispAscii[i] = isprint(buf[i]) ? (char)buf[i] : '.';
    dispAscii[asciiMax] = '\0';

    hasPacket = true;
    drawDisplay();

    Serial.printf("── Packet %lu (%d bytes) ─────────────────\n", packetCount, len);
    Serial.print("  Hex:   ");
    for (int i = 0; i < len; i++) Serial.printf("%02X ", buf[i]);
    Serial.println();
    Serial.printf("  ASCII: \"%s\"\n", dispAscii);
    Serial.printf("  RSSI:  %.1f dBm\n", dispRssi);
    Serial.println("─────────────────────────────────────────\n");
}
