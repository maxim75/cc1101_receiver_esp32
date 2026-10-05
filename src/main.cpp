/**
 * @file    main.cpp
 * @brief   ESP32-S3 + CC1101 + nRF24L01 dual RF receiver with SH1107 OLED display.
 *
 * CC1101  — receives fixed-code 433 MHz remotes (EV1527/PT2262) in async OOK mode;
 *           demodulated pulses on GDO0 are decoded by rc-switch.
 * nRF24L01— receives 2.4 GHz packets on a dedicated HSPI bus.
 * Decoded packet details (hex, ASCII or remote code info, RSSI where available)
 * are shown on the OLED and serial port, and published to MQTT for Home Assistant
 * (remotes: rfgateway/remote JSON; nRF24: nrf/message plain text).
 *
 * Libraries
 *   RadioLib  — jgromes/RadioLib   (CC1101)
 *   rc-switch — sui77/rc-switch    (remote pulse decoding)
 *   RF24      — nrf24/RF24          (nRF24L01)
 *   U8g2      — olikraus/U8g2
 *   PubSubClient — knolleary/PubSubClient (MQTT)
 *   ArduinoJson  — bblanchon/ArduinoJson  (HA discovery payloads)
 *
 * Credentials: copy include/secrets.example.h to include/secrets.h (gitignored).
 *
 * Wiring
 *   Peripheral  Signal  ESP32-S3 GPIO
 *   ─────────────────────────────────
 *   CC1101      SCK     12  ┐
 *               MOSI    11  │ FSPI bus (SPI2)
 *               MISO    13  ┘
 *               CSN     10
 *               GDO0     2   (async OOK data out, CHANGE interrupt)
 *   nRF24L01    SCK     14  ┐
 *               MOSI     1  │ HSPI bus (SPI3)
 *               MISO    21  ┘
 *               CSN      6
 *               CE       5
 *               IRQ      4   (packet interrupt, FALLING)
 *   SH1107      SDA      8
 *               SCL      9
 *   All         VCC     3.3V
 *               GND     GND
 */

#include <RadioLib.h>
#include <RCSwitch.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "include/secrets.h missing: copy include/secrets.example.h to include/secrets.h and fill it in"
#endif

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------

namespace Pin {
    // FSPI bus (SPI2) — CC1101
    constexpr int SCK  = 12;
    constexpr int MISO = 13;
    constexpr int MOSI = 11;

    // CC1101
    constexpr int CC_CS   = 10;
    constexpr int CC_GDO0 =  2;

    // HSPI bus (SPI3) — nRF24L01
    constexpr int NRF_SCK  = 14;
    constexpr int NRF_MOSI =  1;
    constexpr int NRF_MISO = 21;

    // nRF24L01
    constexpr int NRF_CS  =  6;
    constexpr int NRF_CE  =  5;
    constexpr int NRF_IRQ =  4;

    // SH1107 (I2C)
    constexpr int OLED_SDA = 8;
    constexpr int OLED_SCL = 9;
}

namespace Radio {
    constexpr float    FREQUENCY_MHZ  = 433.92f;
    constexpr float    BITRATE_KBPS   =   9.6f;    // finer edge timing in async mode
    constexpr float    DEVIATION_KHZ  =   5.157f;
    constexpr float    RXBW_KHZ       = 270.0f;    // tolerate drift of cheap remotes
    constexpr int      POWER_DBM      =  10;
    constexpr uint32_t REPEAT_GAP_MS  = 300;       // same code within this gap = same press
}

namespace Nrf {
    constexpr uint64_t ADDRESS         = 0xFAB7C2F0E2LL;  // must match transmitter
    constexpr uint32_t HEALTH_CHECK_MS = 1000;            // brownout-reset detection interval
}

namespace Mqtt {
    constexpr const char* CLIENT_ID    = "rfgateway";
    constexpr const char* TOPIC_STATUS = "rfgateway/status";   // retained online/offline (LWT)
    constexpr const char* TOPIC_REMOTE = "rfgateway/remote";   // JSON per remote press
    constexpr const char* TOPIC_NRF    = "nrf/message";        // same as legacy nrf_receiver
    constexpr uint32_t    RETRY_MS     = 5000;
    constexpr uint16_t    BUFFER_SIZE  = 512;                  // fits HA discovery payloads
}

namespace Display {
    constexpr uint8_t WIDTH        = 128;
    constexpr uint8_t HEIGHT       = 128;
    constexpr uint8_t LOG_ROWS     =  10;   // message lines below the header
    constexpr uint8_t LOG_TOP      =  25;   // y of first log row
    constexpr uint8_t ROW_HEIGHT   =  10;
    constexpr uint8_t LOG_CHARS    =  16;   // content chars per row after "HH:MM:SS "
}

namespace Clock {
    constexpr const char* TZ_SYDNEY   = "AEST-10AEDT,M10.1.0,M4.1.0/3";
    constexpr const char* NTP_SERVER  = "pool.ntp.org";
    constexpr time_t      VALID_AFTER = 1700000000;   // earlier = not yet NTP-synced
}

// ---------------------------------------------------------------------------
// Peripherals
// ---------------------------------------------------------------------------
CC1101    cc1101 = new Module(Pin::CC_CS, Pin::CC_GDO0, RADIOLIB_NC, RADIOLIB_NC);
RCSwitch  rcSwitch;
SPIClass  hspi(HSPI);
RF24      nrf24(Pin::NRF_CE, Pin::NRF_CS);
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

// SH1107 panels differ in column mapping; SEEED variant avoids left/right wrap artifacts.
U8G2_SH1107_SEEED_128X128_F_HW_I2C display(
    U8G2_R3, U8X8_PIN_NONE, Pin::OLED_SCL, Pin::OLED_SDA);   // R3 = rotated 270° clockwise

// ---------------------------------------------------------------------------
// Application state
// ---------------------------------------------------------------------------

struct PacketInfo {
    char     source[8]  = {};
    char     ascii[22]  = {};
    float    rssi       = 0.0f;
    uint32_t count      = 0;
    bool     hasRssi    = false;
};

struct LogEntry {
    time_t received = 0;     // 0 = clock was not synced yet
    char   text[24] = {};
};

static PacketInfo lastPacket;
static LogEntry      msgLog[Display::LOG_ROWS];   // [0] = newest
static uint8_t       msgCount      = 0;
static bool          screenDirty   = true;
static time_t        lastDrawnSec  = 0;
static unsigned long lastRemoteCode   = 0;
static uint32_t      lastRemoteSeenMs = 0;
static uint32_t      lastMqttAttemptMs = 0;
static bool          wifiWasUp         = false;
static uint32_t      lastNrfCheckMs    = 0;

// ---------------------------------------------------------------------------
// Display helpers
// ---------------------------------------------------------------------------

static void drawCenteredText(uint8_t y, const char* text) {
    int16_t x = (Display::WIDTH - display.getStrWidth(text)) / 2;
    if (x < 0) x = 0;
    display.drawStr((uint8_t)x, y, text);
}

static void displaySplash(const char* line1, const char* line2 = nullptr) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tr);
    drawCenteredText(54, line1);
    if (line2) drawCenteredText(74, line2);
    display.sendBuffer();
}

static void formatTime(time_t t, char* out, size_t outSize) {
    if (t < Clock::VALID_AFTER) { snprintf(out, outSize, "--:--:--"); return; }
    struct tm local;
    localtime_r(&t, &local);
    strftime(out, outSize, "%H:%M:%S", &local);
}

// Connection indicator: filled dot = up, hollow dot = down
static void drawStatus(uint8_t x, uint8_t baseline, const char* label, bool up) {
    if (up) display.drawDisc(x + 3, baseline - 4, 3);
    else    display.drawCircle(x + 3, baseline - 4, 3);
    display.drawStr(x + 9, baseline, label);
}

static void addMessage(const char* text) {
    memmove(&msgLog[1], &msgLog[0], sizeof(LogEntry) * (Display::LOG_ROWS - 1));
    const time_t now = time(nullptr);
    msgLog[0].received = now >= Clock::VALID_AFTER ? now : 0;
    snprintf(msgLog[0].text, sizeof(msgLog[0].text), "%s", text);
    if (msgCount < Display::LOG_ROWS) msgCount++;
    screenDirty = true;
}

// Header (IP, Wi-Fi/MQTT status, clock) above a rolling log, newest row inverted.
static void drawScreen() {
    char timeStr[12];
    display.clearBuffer();
    display.setFontMode(1);   // transparent glyph background for inverted rows

    display.setFont(u8g2_font_6x10_tr);
    const bool wifiUp = WiFi.status() == WL_CONNECTED;
    display.drawStr(0, 9, wifiUp ? WiFi.localIP().toString().c_str() : "Wi-Fi connecting...");
    drawStatus(0,  20, "WiFi", wifiUp);
    drawStatus(39, 20, "MQTT", mqtt.connected());
    formatTime(time(nullptr), timeStr, sizeof(timeStr));
    display.drawStr(Display::WIDTH - display.getStrWidth(timeStr), 20, timeStr);
    display.drawHLine(0, 23, Display::WIDTH);

    display.setFont(u8g2_font_5x8_tr);
    if (msgCount == 0) drawCenteredText(Display::LOG_TOP + 4 * Display::ROW_HEIGHT, "Listening...");

    for (uint8_t i = 0; i < msgCount; i++) {
        const uint8_t top = Display::LOG_TOP + i * Display::ROW_HEIGHT;
        if (i == 0) {
            display.drawBox(0, top, Display::WIDTH, Display::ROW_HEIGHT);
            display.setDrawColor(0);
        }

        char content[Display::LOG_CHARS + 1];
        snprintf(content, sizeof(content), "%s", msgLog[i].text);
        if (strlen(msgLog[i].text) > Display::LOG_CHARS) {
            content[Display::LOG_CHARS - 2] = '.';
            content[Display::LOG_CHARS - 1] = '.';
        }
        formatTime(msgLog[i].received, timeStr, sizeof(timeStr));
        display.drawStr(1,  top + 8, timeStr);
        display.drawStr(46, top + 8, content);   // 9 chars * 5 px + 1

        display.setDrawColor(1);
    }

    display.sendBuffer();
}

// ---------------------------------------------------------------------------
// Radio helpers
// ---------------------------------------------------------------------------

// Full nRF24 configuration. Also used to recover after a supply brownout resets
// the module to power-on defaults (2 Mbps, powered down), which stops reception.
static bool initNrf24() {
    if (!nrf24.begin(&hspi)) return false;
    nrf24.setDataRate(RF24_250KBPS);
    nrf24.openReadingPipe(0, Nrf::ADDRESS);
    nrf24.setPALevel(RF24_PA_MAX, true);
    nrf24.startListening();
    return true;
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
    pkt.rssi    = rssi;
    pkt.hasRssi = hasRssi;
    strncpy(pkt.source, source, sizeof(pkt.source) - 1);
    pkt.source[sizeof(pkt.source) - 1] = '\0';
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

static void logRemoteToSerial(const PacketInfo& pkt, unsigned long code,
                              unsigned int bits, unsigned int protocol, unsigned int pulseUs) {
    Serial.printf("── [%s] Press %lu ──────────\n", pkt.source, pkt.count);
    Serial.printf("  Code:     %lu (0x%lX)\n", code, code);
    Serial.printf("  Bits:     %u\n", bits);
    Serial.printf("  Protocol: %u\n", protocol);
    Serial.printf("  Pulse:    %u us\n", pulseUs);
    Serial.printf("  RSSI:     %.1f dBm\n", pkt.rssi);
    Serial.println("─────────────────────────────────────────\n");
}

// ---------------------------------------------------------------------------
// MQTT / Home Assistant
// ---------------------------------------------------------------------------

// Announce a sensor via Home Assistant MQTT discovery (retained).
// valueTemplate == nullptr means the state topic carries plain text.
static void publishDiscoverySensor(const char* objectId, const char* name,
                                   const char* stateTopic, const char* valueTemplate) {
    char uniqueId[48];
    snprintf(uniqueId, sizeof(uniqueId), "%s_%s", Mqtt::CLIENT_ID, objectId);

    StaticJsonDocument<512> doc;
    doc["name"]               = name;
    doc["unique_id"]          = uniqueId;
    doc["state_topic"]        = stateTopic;
    doc["availability_topic"] = Mqtt::TOPIC_STATUS;
    if (valueTemplate) {
        doc["value_template"]        = valueTemplate;
        doc["json_attributes_topic"] = stateTopic;
    }
    JsonObject device = doc.createNestedObject("device");
    device.createNestedArray("identifiers").add(Mqtt::CLIENT_ID);
    device["name"] = "RF Gateway";

    char topic[80];
    snprintf(topic, sizeof(topic), "homeassistant/sensor/%s/%s/config", Mqtt::CLIENT_ID, objectId);
    char payload[Mqtt::BUFFER_SIZE];
    const size_t len = serializeJson(doc, payload, sizeof(payload));
    mqtt.publish(topic, (const uint8_t*)payload, len, true);
}

// Non-blocking Wi-Fi/MQTT upkeep; called every loop(). A connect attempt to an
// unreachable broker can block for a few seconds — radio events then are lost.
static void mqttMaintain() {
    const bool wifiUp = WiFi.status() == WL_CONNECTED;
    if (wifiUp != wifiWasUp) {
        wifiWasUp = wifiUp;
        if (wifiUp) Serial.printf("[WIFI] connected, IP %s\n", WiFi.localIP().toString().c_str());
        else        Serial.println("[WIFI] disconnected");
    }

    if (mqtt.connected()) { mqtt.loop(); return; }
    if (!wifiUp) return;

    const uint32_t now = millis();
    if (lastMqttAttemptMs != 0 && now - lastMqttAttemptMs < Mqtt::RETRY_MS) return;
    lastMqttAttemptMs = now;

    if (!mqtt.connect(Mqtt::CLIENT_ID, MQTT_USER, MQTT_PASSWORD,
                      Mqtt::TOPIC_STATUS, 0, true, "offline")) {
        Serial.printf("[MQTT] connect failed, rc=%d\n", mqtt.state());
        return;
    }
    Serial.println("[MQTT] connected");
    mqtt.publish(Mqtt::TOPIC_STATUS, "online", true);
    publishDiscoverySensor("remote_code",  "Last remote code",  Mqtt::TOPIC_REMOTE, "{{ value_json.code }}");
    publishDiscoverySensor("nrf24_packet", "Last nRF24 packet", Mqtt::TOPIC_NRF,    nullptr);
}

static void publishRemote(unsigned long code, unsigned int bits, unsigned int protocol,
                          unsigned int pulseUs, float rssi) {
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"code\":%lu,\"bits\":%u,\"protocol\":%u,\"pulse\":%u,\"rssi\":%.0f}",
             code, bits, protocol, pulseUs, rssi);
    mqtt.publish(Mqtt::TOPIC_REMOTE, payload);
}

// Same format as the legacy nrf_receiver firmware: raw text up to the first NUL.
static void publishNrf(const uint8_t* data, int len) {
    char text[33] = {};
    memcpy(text, data, min(len, 32));
    mqtt.publish(Mqtt::TOPIC_NRF, text);
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

    SPI.begin(Pin::SCK, Pin::MISO, Pin::MOSI);
    hspi.begin(Pin::NRF_SCK, Pin::NRF_MISO, Pin::NRF_MOSI, Pin::NRF_CS);

    // --- CC1101 ---
    int state = cc1101.begin(
        Radio::FREQUENCY_MHZ,
        Radio::BITRATE_KBPS,
        Radio::DEVIATION_KHZ,
        Radio::RXBW_KHZ,
        Radio::POWER_DBM);

    if (state != RADIOLIB_ERR_NONE) {
        char errMsg[24];
        snprintf(errMsg, sizeof(errMsg), "CC1101 err: %d", state);
        Serial.printf("[ERROR] CC1101 init failed: %d\n", state);
        displaySplash("CC1101 FAILED", errMsg);
        while (true) delay(1000);
    }

    cc1101.setOOK(true);
    cc1101.receiveDirectAsync();    // GDO0 now outputs raw demodulated OOK data
    rcSwitch.enableReceive(digitalPinToInterrupt(Pin::CC_GDO0));

    Serial.println("[OK] CC1101 ready");
    Serial.printf("  Frequency:  %.2f MHz\n",  Radio::FREQUENCY_MHZ);
    Serial.printf("  RX BW:      %.0f kHz\n",  Radio::RXBW_KHZ);
    Serial.println("  Modulation: OOK async | Decoder: rc-switch");

    // --- nRF24L01 ---
    if (!initNrf24()) {
        Serial.println("[ERROR] nRF24 init failed");
        displaySplash("nRF24 FAILED", "check wiring");
        while (true) delay(1000);
    }

    Serial.println("[OK] nRF24 ready");
    Serial.printf("  Address:    0x%010llX\n", Nrf::ADDRESS);
    Serial.println("  Data rate:  250 kbps | PA: MAX");

    // --- Wi-Fi / MQTT (connection happens in loop() so radios run immediately) ---
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setBufferSize(Mqtt::BUFFER_SIZE);
    mqtt.setSocketTimeout(2);
    Serial.printf("[..] Wi-Fi \"%s\", MQTT %s:%d\n", WIFI_SSID, MQTT_HOST, MQTT_PORT);
    configTzTime(Clock::TZ_SYDNEY, Clock::NTP_SERVER);   // syncs once Wi-Fi is up

    Serial.println("\n[LISTENING]\n");

    drawScreen();  // shows "Listening..."
}

void loop() {
    mqttMaintain();

    if (rcSwitch.available()) {
        const unsigned long code     = rcSwitch.getReceivedValue();
        const unsigned int  bits     = rcSwitch.getReceivedBitlength();
        const unsigned int  protocol = rcSwitch.getReceivedProtocol();
        const unsigned int  pulseUs  = rcSwitch.getReceivedDelay();
        rcSwitch.resetAvailable();

        // Remotes repeat each frame many times per press; report a press once
        const uint32_t now = millis();
        const bool repeat = code == lastRemoteCode && now - lastRemoteSeenMs < Radio::REPEAT_GAP_MS;
        lastRemoteCode   = code;
        lastRemoteSeenMs = now;

        if (code != 0 && !repeat) {
            fillPacket(lastPacket, "Remote", nullptr, 0, cc1101.getRSSI(), true);
            char text[sizeof(LogEntry::text)];
            snprintf(text, sizeof(text), "RF %lu", code);
            addMessage(text);
            logRemoteToSerial(lastPacket, code, bits, protocol, pulseUs);
            publishRemote(code, bits, protocol, pulseUs, lastPacket.rssi);
        }
    }

    // A brownout reset restores the 2 Mbps default; re-apply our configuration
    if (millis() - lastNrfCheckMs >= Nrf::HEALTH_CHECK_MS) {
        lastNrfCheckMs = millis();
        if (nrf24.getDataRate() != RF24_250KBPS) {
            Serial.printf("[WARN] nRF24 reset detected, %s\n",
                          initNrf24() ? "reinitialised" : "reinit failed");
        }
    }

    if (nrf24.available()) {
        uint8_t buf[32] = {};
        uint8_t len = nrf24.getDynamicPayloadSize();
        if (len == 0 || len > sizeof(buf)) len = nrf24.getPayloadSize();
        if (len == 0 || len > sizeof(buf)) len = (uint8_t)sizeof(buf);
        nrf24.read(buf, len);

        fillPacket(lastPacket, "nRF24", buf, len, 0.0f, false);
        // Display text ends at the first NUL, like the MQTT payload (hides padding)
        char text[sizeof(LogEntry::text)];
        buildAsciiString(buf, (int)strnlen((const char*)buf, len), text, sizeof(text));
        addMessage(text);
        logPacketToSerial(lastPacket, buf, len);
        publishNrf(buf, len);
    }

    // Redraw on new messages and once per second for the clock and status
    const time_t nowSec = time(nullptr);
    if (screenDirty || nowSec != lastDrawnSec) {
        screenDirty  = false;
        lastDrawnSec = nowSec;
        drawScreen();
    }
}
