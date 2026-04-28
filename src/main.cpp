/**
 * ESP32-S3 + CC1101 Receiver
 * Library: RadioLib (jgromes/RadioLib)
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
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <RadioLib.h>
#include <SPI.h>

#define CC1101_SCK   40
#define CC1101_MISO  42
#define CC1101_MOSI  41
#define CC1101_CS    38
#define CC1101_GDO0   2

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

    // preambleLength is in BITS (16 = 2 bytes, CC1101 minimum)
    int state = radio.begin(433.92, 4.8, 5.157, 203.0, 10, 16);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[ERROR] CC1101 init failed: %d\n", state);
        while (true) delay(1000);
    }
    Serial.println("[OK] CC1101 detected");

    radio.setOOK(true);                  // match ATtiny TX: setModulation(2)
    radio.setSyncWord(0xD3, 0x91);       // CC1101 default, confirmed from capture
    radio.setCrcFiltering(true);         // drop bad-CRC packets

    radio.setGdo0Action(onReceive, RISING);
    radio.startReceive();

    Serial.println("  Modulation: OOK");
    Serial.println("  Frequency:  433.92 MHz");
    Serial.println("  Data rate:  4.8 kbps");
    Serial.println("  Sync word:  0xD391");
    Serial.println("  CRC:        enabled");
    Serial.println("\n[LISTENING]\n");
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

    Serial.printf("── Packet (%d bytes) ─────────────────────\n", len);

    Serial.print("  Hex:   ");
    for (int i = 0; i < len; i++) Serial.printf("%02X ", buf[i]);
    Serial.println();

    Serial.print("  ASCII: \"");
    for (int i = 0; i < len; i++)
        Serial.print(isprint(buf[i]) ? (char)buf[i] : '.');
    Serial.println("\"");

    Serial.printf("  RSSI:  %.1f dBm\n", radio.getRSSI());
    Serial.println("─────────────────────────────────────────\n");
}
