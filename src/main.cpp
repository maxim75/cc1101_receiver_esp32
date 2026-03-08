#include <Arduino.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

#define CC1101_FREQ   433.92  // MHz — change to 315.0 or 868.0 as needed

byte rxBuffer[64];

void setup() {
  Serial.begin(115200);
  Serial.println("Starting CC1101 Receiver...");

  ELECHOUSE_cc1101.setSpiPin(12, 13, 11, 10); // SCK, MISO, MOSI, SS
  ELECHOUSE_cc1101.setGDO0(2);               // GDO0 on GPIO 2

  if (ELECHOUSE_cc1101.getCC1101()) {
    Serial.println("CC1101 connection OK");
  } else {
    Serial.println("CC1101 connection Error — check wiring");
  }

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setCRC_AF(true);  // flush RX FIFO on CRC failure
  ELECHOUSE_cc1101.setMHZ(CC1101_FREQ);
  ELECHOUSE_cc1101.SetRx();

  Serial.println("Receiver ready. Waiting for data...");
}

void loop() {
  if (ELECHOUSE_cc1101.CheckReceiveFlag()) {
    int len = ELECHOUSE_cc1101.ReceiveData(rxBuffer);

    if (len == 0) { ELECHOUSE_cc1101.SetRx(); return; }  // CRC fail / noise

    Serial.print("Received command [");
    Serial.print(len);
    Serial.print(" bytes]: ");
    for (int i = 0; i < len; i++) {
      if (rxBuffer[i] < 0x10) Serial.print('0');
      Serial.print(rxBuffer[i], HEX);
      Serial.print(' ');
    }
    Serial.println();

    // Print raw text if all bytes are printable
    bool printable = true;
    for (int i = 0; i < len; i++) {
      if (!isPrintable(rxBuffer[i])) { printable = false; break; }
    }
    if (printable) {
      Serial.print("As text: ");
      for (int i = 0; i < len; i++) Serial.print((char)rxBuffer[i]);
      Serial.println();
    }

    Serial.print("RSSI: ");
    Serial.print(ELECHOUSE_cc1101.getRssi());
    Serial.println(" dBm");

    ELECHOUSE_cc1101.SetRx();  // re-arm receiver
  }
}