/**
 * ESP32-S3 + CC1101 Receiver
 * Library: RadioLib (jgromes/RadioLib)
 *
 * ── Wiring (ESP32-S3 DevKitC-1 → CC1101) ────────────────────────────────────
 *   CC1101    ESP32-S3    Notes
 *   VCC       3.3V        Do NOT use 5V — will destroy CC1101
 *   GND       GND
 *   SCK       GPIO 40
 *   MOSI      GPIO 41
 *   MISO      GPIO 42
 *   CSN       GPIO 38     SPI chip select (active LOW)
 *   GDO0      GPIO 2      Packet interrupt
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <RadioLib.h>
#include <SPI.h>

#define CC1101_SCK   40
#define CC1101_MISO  42
#define CC1101_MOSI  41
#define CC1101_CS    38
#define CC1101_GDO0   2

// Use the global SPI object (FSPI/SPI2_HOST). Custom pins go through the GPIO
// matrix regardless, and the global SPI is what ELECHOUSE used — known working.
CC1101 radio = new Module(CC1101_CS, CC1101_GDO0, RADIOLIB_NC, RADIOLIB_NC);

volatile bool receivedFlag = false;

void IRAM_ATTR onReceive() {
    receivedFlag = true;
}

void setup() {
    Serial.begin(115200);
    for (uint32_t t = millis(); !Serial && (millis() - t < 3000); ) delay(10);
    Serial.println("\n==============================");
    Serial.println("  ESP32 CC1101 Receiver");
    Serial.println("==============================");

    SPI.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);

    // begin(freq MHz, bitrate kbps, freqDev kHz, rxBw kHz, power dBm, preamble bits)
    // preamble unit is BITS: 16 = 2 bytes (CC1101 minimum, matches ATtiny TX)
    int state = radio.begin(433.92, 4.8, 5.157, 101.5625, 10, 16);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] CC1101 init failed, code %d\n", state);
        while (true) delay(1000);
    }
    Serial.println("[OK] CC1101 detected");

    // ASK/OOK modulation (matches ATtiny transmitter)
    radio.setOOK(true);

    // Sync word 0xD3 0x91 (CC1101 default; matches captured preamble pattern)
    radio.setSyncWord(0xD3, 0x91);

    // CRC filtering: bad-CRC packets discarded automatically
    radio.setCrcFiltering(true);

    radio.setGdo0Action(onReceive, RISING);
    radio.startReceive();

    Serial.println("\n── Radio Configuration ──────────────────");
    Serial.println("  Frequency:  433.92 MHz");
    Serial.println("  Data rate:  4.8 kbps");
    Serial.println("  Modulation: ASK/OOK");
    Serial.println("  Sync word:  0xD391");
    Serial.println("  CRC:        Enabled");
    Serial.println("─────────────────────────────────────────\n");
    Serial.println("[LISTENING] Waiting for packets...\n");
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
        Serial.printf("[WARN] readData error: %d\n", state);
        return;
    }

    Serial.printf("── Packet received (%d bytes) ──────────────\n", len);
    Serial.print("  Raw hex: ");
    for (int i = 0; i < len; i++) Serial.printf("%02X ", buf[i]);
    Serial.println();

    Serial.printf("  RSSI: %.1f dBm\n", radio.getRSSI());
    Serial.printf("  LQI:  %d\n", radio.getLQI());

    if (len >= 2) {
        uint8_t payload_len = buf[0];
        if (payload_len > 0 && payload_len < len) {
            Serial.printf("  Payload (%d bytes): ", payload_len);
            for (int i = 1; i <= payload_len && i < len; i++) Serial.printf("%02X ", buf[i]);
            Serial.println();
            Serial.print("  As ASCII: \"");
            for (int i = 1; i <= payload_len && i < len; i++)
                Serial.print(isprint(buf[i]) ? (char)buf[i] : '.');
            Serial.println("\"");
        }
    }

    Serial.println("────────────────────────────────────────\n");
}
