/**
 * ESP32-S3 + CC1101 Receiver
 * Receives packets from ATtiny3226 transmitter (4-byte payload 'ABCD').
 *
 * Transmitter uses ELECHOUSE library with:
 *   setCCMode(1), setMHZ(433.92), setModulation(2) ASK/OOK,
 *   setSyncMode(2) 16/16 sync, setCrc(true), setDRate(4.80 kbps)
 *   Default CC1101 sync word: 0xD3 0x91
 *
 * ── Wiring (ESP32-S3 DevKitC-1 → CC1101) ────────────────────────────────────
 *
 *   CC1101 Pin   ESP32-S3 Pin   Notes
 *   ─────────────────────────────────────────────────────
 *   VCC          3.3V           Do NOT use 5V — will destroy CC1101
 *   GND          GND
 *   SCK          GPIO 12        SPI clock (FSPI)
 *   MOSI         GPIO 11        SPI data to CC1101 (FSPI)
 *   MISO         GPIO 13        SPI data from CC1101 (FSPI)
 *   CSN          GPIO 10        SPI chip select (active LOW)
 *   GDO0         GPIO 2         Packet-received interrupt (RadioLib)
 *   GDO2         (NC)           Not used
 */

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#define CC1101_SCK   12
#define CC1101_MISO  13
#define CC1101_MOSI  11
#define CC1101_CS    10
#define CC1101_GDO0   2

SPIClass spi(FSPI);
CC1101 radio = new Module(CC1101_CS, CC1101_GDO0, RADIOLIB_NC, RADIOLIB_NC, spi);

volatile bool packetReceived = false;
volatile uint32_t isrCount = 0;
IRAM_ATTR void onReceive() { packetReceived = true; isrCount++; }

static uint32_t pktCount = 0;

#define RADIO_CHECK(call, name) do { \
    int _s = (call); \
    if (_s != RADIOLIB_ERR_NONE) { \
        Serial.printf("[ERROR] %s failed, code %d\n", name, _s); \
        while (true) delay(1000); \
    } \
} while (0)

void setup() {
    Serial.begin(115200);
    for (uint32_t t = millis(); !Serial && (millis() - t < 3000); ) delay(10);
    Serial.println("\n==============================");
    Serial.println("  ESP32 CC1101 Receiver");
    Serial.println("==============================");

    spi.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);

    RADIO_CHECK(radio.begin(), "begin");
    Serial.println("[OK] CC1101 detected");

    // Match ATtiny3226 ELECHOUSE transmitter exactly
    RADIO_CHECK(radio.setFrequency(433.92),       "setFrequency");
    RADIO_CHECK(radio.setBitRate(4.8),             "setBitRate");
    RADIO_CHECK(radio.setOOK(true),                "setOOK");
    RADIO_CHECK(radio.setRxBandwidth(135.0),       "setRxBandwidth");
    RADIO_CHECK(radio.setSyncWord(0xD3, 0x91),     "setSyncWord");

    Serial.println("  Frequency:  433.92 MHz");
    Serial.println("  Modulation: ASK/OOK");
    Serial.println("  Bit rate:   4.8 kbps");
    Serial.println("  RX BW:      135 kHz");
    Serial.println("  Sync word:  0xD391");

    // Disable CRC auto-flush so packets with bad CRC still trigger GDO0.
    // Without this, a CRC mismatch silently drops the packet and the ISR never fires.
    radio.setCrcFiltering(false);

    // Ensure NRZ encoding (no data whitening) — ELECHOUSE transmitter defaults to no whitening.
    radio.setEncoding(RADIOLIB_ENCODING_NRZ);

    radio.setPacketReceivedAction(onReceive);
    RADIO_CHECK(radio.startReceive(), "startReceive");

    Serial.println("[LISTENING] Waiting for packets from ATtiny3226...\n");
}

static unsigned long lastHeartbeatMs = 0;

void loop() {
    unsigned long now = millis();
    if (now - lastHeartbeatMs >= 10000) {
        lastHeartbeatMs = now;
        Serial.printf("[HEARTBEAT] RSSI=%.1f dBm  ISR fires=%lu\n",
                      radio.getRSSI(), (unsigned long)isrCount);
    }

    if (!packetReceived) return;
    packetReceived = false;

    int len = radio.getPacketLength();
    if (len <= 0 || len > 64) {
        Serial.printf("[DBG] bad len=%d — discarded\n", len);
        radio.startReceive();
        return;
    }

    byte buf[64];
    int state = radio.readData(buf, len);
    if (state == RADIOLIB_ERR_CRC_MISMATCH) {
        Serial.println("[WARN] CRC mismatch — discarding");
        radio.startReceive();
        return;
    }
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[WARN] readData code %d\n", state);
        radio.startReceive();
        return;
    }

    pktCount++;
    float rssi = radio.getRSSI();

    Serial.printf("── Packet #%lu (%d bytes) ──────────────────\n",
                  (unsigned long)pktCount, len);
    Serial.print("  Hex:   ");
    for (int i = 0; i < len; i++) Serial.printf("%02X ", buf[i]);
    Serial.println();
    Serial.print("  ASCII: ");
    for (int i = 0; i < len; i++) {
        char c = (char)buf[i];
        Serial.print(isprint(c) ? c : '.');
    }
    Serial.println();
    Serial.printf("  RSSI:  %.1f dBm\n\n", rssi);

    radio.startReceive();
}
