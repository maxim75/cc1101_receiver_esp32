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
    constexpr uint64_t ADDRESS = 0xFAB7C2F0E2LL;  // must match transmitter
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
    constexpr uint8_t WIDTH         = 128;
    constexpr uint8_t HEIGHT        = 128;
    constexpr int     HEX_MAX_BYTES =   7;    // bytes shown on OLED before "..."
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
    U8G2_R0, U8X8_PIN_NONE, Pin::OLED_SCL, Pin::OLED_SDA);

// ---------------------------------------------------------------------------
// Application state
// ---------------------------------------------------------------------------

struct PacketInfo {
    char     source[8]  = {};
    char     hex[48]    = {};
    char     ascii[22]  = {};
    char     info[22]   = {};    // shown instead of ASCII when set (remote code info)
    float    rssi       = 0.0f;
    uint32_t count      = 0;
    bool     valid      = false;
    bool     hasRssi    = false;
};

static PacketInfo lastPacket;
static unsigned long lastRemoteCode   = 0;
static uint32_t      lastRemoteSeenMs = 0;
static uint32_t      lastMqttAttemptMs = 0;
static bool          wifiWasUp         = false;

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

static void displayPacket(const PacketInfo& pkt) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tr);

    // Header: source on left, packet counter on right
    display.drawStr(0, 12, pkt.valid ? pkt.source : "Dual RX");
    if (pkt.count > 0) {
        char counter[12];
        snprintf(counter, sizeof(counter), "#%lu", pkt.count);
        display.drawStr(Display::WIDTH - display.getStrWidth(counter), 12, counter);
    }
    display.drawHLine(0, 16, Display::WIDTH);

    if (!pkt.valid) {
        drawCenteredText(68, "Listening...");
    } else {
        constexpr uint8_t textX = 4;
        uint8_t y = 36;
        if (pkt.hasRssi) {
            char rssiLine[20];
            snprintf(rssiLine, sizeof(rssiLine), "RSSI: %.0f dBm", pkt.rssi);
            display.drawStr(textX, y, rssiLine);
            y += 20;
        }
        display.drawStr(textX, y, pkt.hex);   y += 20;

        if (pkt.info[0]) {
            display.drawStr(textX, y, pkt.info);
        } else {
            char asciiLine[24];
            snprintf(asciiLine, sizeof(asciiLine), "\"%s\"", pkt.ascii);
            display.drawStr(textX, y, asciiLine);
        }
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
    pkt.info[0] = '\0';
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
    if (!nrf24.begin(&hspi)) {
        Serial.println("[ERROR] nRF24 init failed");
        displaySplash("nRF24 FAILED", "check wiring");
        while (true) delay(1000);
    }
    nrf24.setDataRate(RF24_250KBPS);
    nrf24.openReadingPipe(0, Nrf::ADDRESS);
    nrf24.setPALevel(RF24_PA_MAX, true);
    nrf24.startListening();

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

    Serial.println("\n[LISTENING]\n");

    displayPacket(lastPacket);  // shows "Listening..."
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
            // Code as big-endian bytes so the hex view reads like the code value
            uint8_t buf[sizeof(code)];
            const int len = min((int)((bits + 7) / 8), (int)sizeof(buf));
            for (int i = 0; i < len; i++) buf[i] = (uint8_t)(code >> (8 * (len - 1 - i)));

            fillPacket(lastPacket, "Remote", buf, len, cc1101.getRSSI(), true);
            snprintf(lastPacket.info, sizeof(lastPacket.info), "P%u %ubit %uus", protocol, bits, pulseUs);
            displayPacket(lastPacket);
            logRemoteToSerial(lastPacket, code, bits, protocol, pulseUs);
            publishRemote(code, bits, protocol, pulseUs, lastPacket.rssi);
        }
    }

    if (nrf24.available()) {
        uint8_t buf[32] = {};
        uint8_t len = nrf24.getDynamicPayloadSize();
        if (len == 0 || len > sizeof(buf)) len = nrf24.getPayloadSize();
        if (len == 0 || len > sizeof(buf)) len = (uint8_t)sizeof(buf);
        nrf24.read(buf, len);

        fillPacket(lastPacket, "nRF24", buf, len, 0.0f, false);
        displayPacket(lastPacket);
        logPacketToSerial(lastPacket, buf, len);
        publishNrf(buf, len);
    }
}
