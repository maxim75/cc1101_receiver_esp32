/**
 * ESP32-S3 + CC1101 Receiver
 * Configured to receive packet: AA AA D391 04 41424344 3F4B
 *
 * Library: ELECHOUSE_CC1101_SRC_DRV
 * Install: PlatformIO Library Manager → lsatan/SmartRC-CC1101-Driver-Lib
 * https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
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
 *   GDO0         GPIO 2         Packet-received interrupt / TX signal
 *   GDO2         (NC)           Not used in this sketch
 *
 *   ⚠ GPIO 19/20 are USB D-/D+ on ESP32-S3 — do not use for SPI.
 *   ⚠ GPIO 22-25 do not exist on ESP32-S3.
 *
 *   ⚠ Add a 100 nF decoupling capacitor between VCC and GND, close to CC1101.
 *   ⚠ Keep SPI traces short to minimise noise at 433 MHz.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <SPI.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define CC1101_SCK   12
#define CC1101_MISO  13
#define CC1101_MOSI  11
#define CC1101_CS    10
#define CC1101_GDO0   2

// ── Signal parameters (matched to captured packet) ───────────────────────────
#define RF_FREQUENCY  433.92   // MHz — adjust to your actual frequency
#define RF_CHANNEL    0        // CHANNR register
#define RF_BAUD       4800.0   // bps — adjust if your ATtiny uses different rate
#define RF_DEVN       5.157    // kHz FSK deviation (CC1101 default, DEVIATN=0x47)
#define RF_BW         101.5625 // kHz RX filter bandwidth (MDMCFG4 default)
#define RF_POWER      10       // TX power (dBm) — not used for RX but required by lib
#define SYNC_WORD_1   0xD3     // Sync byte 1 (MSB) — from captured packet
#define SYNC_WORD_2   0x91     // Sync byte 2 (LSB) — from captured packet
#define MAX_PACKET    61       // CC1101 RXFIFO max usable size

// ── CRC-16 verification (polynomial 0x8005, init 0xFFFF) ─────────────────────
uint16_t crc16_cc1101(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x8005) : (crc << 1);
        }
    }
    return crc;
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    // Wait up to 3 s for USB CDC host to connect (ESP32-S3 native USB)
    for (uint32_t t = millis(); !Serial && (millis() - t < 3000); ) delay(10);
    Serial.println("\n==============================");
    Serial.println("  ESP32 CC1101 Receiver");
    Serial.println("==============================");

    // Set CS and GDO0 pins before init
    ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS);
    ELECHOUSE_cc1101.setGDO0(CC1101_GDO0);  // GDO2 not used — avoids touching strapping GPIO 0

    if (!ELECHOUSE_cc1101.getCC1101()) {
        Serial.println("[ERROR] CC1101 not found! Check SPI wiring.");
        Serial.println("  → Verify MISO/MOSI not swapped");
        Serial.println("  → Verify 3.3V power with decoupling caps");
        while (true) delay(1000);
    }
    Serial.println("[OK] CC1101 detected");

    // ── Core radio config ─────────────────────────────────────────────────
    ELECHOUSE_cc1101.Init();
    // ELECHOUSE_cc1101.setMHZ(RF_FREQUENCY);
    // ELECHOUSE_cc1101.setChannel(RF_CHANNEL);
    // ELECHOUSE_cc1101.setModulation(2);            // 2 = GFSK (CC1101 default)
    // ELECHOUSE_cc1101.setDRate(RF_BAUD);
    // ELECHOUSE_cc1101.setDeviation(RF_DEVN);
    // ELECHOUSE_cc1101.setRxBW(RF_BW);
    // ELECHOUSE_cc1101.setSyncWord(SYNC_WORD_1, SYNC_WORD_2);
    // ELECHOUSE_cc1101.setPktFormat(0);             // Normal packet mode
    // ELECHOUSE_cc1101.setLengthConfig(1);          // Variable packet length mode
    // ELECHOUSE_cc1101.setCrc(1);                   // CRC enabled — must match TX side
    // ELECHOUSE_cc1101.setCRC_AF(1);                // Auto-flush packets with bad CRC
    // ELECHOUSE_cc1101.setAdrChk(0);               // No address check (set 1 if TX sends address)
    // ELECHOUSE_cc1101.setWhiteData(0);             // Data whitening OFF (match ATtiny)
    // ELECHOUSE_cc1101.setManchester(0);            // Manchester encoding OFF (match ATtiny)
    // ELECHOUSE_cc1101.setPA(RF_POWER);
    ELECHOUSE_cc1101.setCCMode(1);
    ELECHOUSE_cc1101.setMHZ(433.92);
    ELECHOUSE_cc1101.setModulation(2);   // ASK/OOK
    ELECHOUSE_cc1101.setSyncMode(2);     // no sync
    ELECHOUSE_cc1101.setCrc(true);      // no CRC
    ELECHOUSE_cc1101.setDRate(4.80);

    // Preamble: 2 bytes of 0xAA (as seen in capture)
    // Default preamble length register (MDMCFG1) = 2 bytes — no change needed

    Serial.println("\n── Radio Configuration ──────────────────");
    Serial.printf("  Frequency:  %.2f MHz\n",    RF_FREQUENCY);
    Serial.printf("  Data rate:  %.0f bps\n",    RF_BAUD);
    Serial.printf("  Deviation:  %.3f kHz\n",   RF_DEVN);
    Serial.printf("  BW filter:  %.4f kHz\n",   RF_BW);
    Serial.printf("  Sync word:  0x%02X%02X\n", SYNC_WORD_1, SYNC_WORD_2);
    Serial.println("  Modulation: 2-FSK");
    Serial.println("  CRC:        Enabled");
    Serial.println("─────────────────────────────────────────\n");

    ELECHOUSE_cc1101.SetRx();  // Enter RX mode
    Serial.println("[LISTENING] Waiting for packets...\n");
}

// ── Main loop ────────────────────────────────────────────────────────────────
void loop() {
    if (!ELECHOUSE_cc1101.CheckReceiveFlag()) return;  // No packet yet

    uint8_t buf[MAX_PACKET];
    int len = ELECHOUSE_cc1101.ReceiveData(buf);

    if (len <= 0) {
        Serial.println("[WARN] Empty or invalid packet received");
        ELECHOUSE_cc1101.SetRx();
        return;
    }

    // ── Print raw bytes ───────────────────────────────────────────────────
    Serial.printf("── Packet received (%d bytes) ──────────────\n", len);
    Serial.print("  Raw hex: ");
    for (int i = 0; i < len; i++) Serial.printf("%02X ", buf[i]);
    Serial.println();

    // ── RSSI & LQI (appended by CC1101 when APPEND_STATUS is on) ─────────
    int rssi = ELECHOUSE_cc1101.getRssi();
    int lqi  = ELECHOUSE_cc1101.getLqi();
    Serial.printf("  RSSI: %d dBm\n", rssi);
    Serial.printf("  LQI:  %d  (lower = better, <30 ideal)\n", lqi);

    // ── Manual CRC verification (length byte + payload) ──────────────────
    if (len >= 3) {
        uint16_t rx_crc   = ((uint16_t)buf[len-2] << 8) | buf[len-1];
        uint16_t calc_crc = crc16_cc1101(buf, len - 2);

        Serial.printf("  CRC received:   0x%04X\n", rx_crc);
        Serial.printf("  CRC calculated: 0x%04X\n", calc_crc);
        Serial.printf("  CRC valid:      %s\n", (rx_crc == calc_crc) ? "YES ✓" : "NO ✗");

        // ── Decode payload (buf[0] = length, buf[1..n-2] = payload) ──────
        uint8_t payload_len = buf[0];
        if (payload_len <= len - 3 && payload_len > 0) {
            Serial.printf("  Payload (%d bytes): ", payload_len);
            for (int i = 1; i <= payload_len; i++) Serial.printf("%02X ", buf[i]);
            Serial.println();

            // Print as ASCII if printable
            Serial.print("  As ASCII: \"");
            for (int i = 1; i <= payload_len; i++) {
                Serial.print(isprint(buf[i]) ? (char)buf[i] : '.');
            }
            Serial.println("\"");
        }
    }

    Serial.println("────────────────────────────────────────\n");

    ELECHOUSE_cc1101.SetRx();  // Re-arm receiver for next packet
}
