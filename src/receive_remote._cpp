#include <Arduino.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

#define CC1101_FREQ     433.92  // MHz
#define GDO0_PIN        2

// Set to 1 to print raw pulse timings — useful for tuning constants
#define PT2262_DEBUG    1

// PT2262 pulse timing (µs). T ≈ 350µs, but varies between remotes.
#define T_SHORT_MIN  150
#define T_SHORT_MAX  900
#define T_LONG_MIN   750
#define T_LONG_MAX   3200
#define T_SYNC_MIN   8000   // 31T sync LOW > 8 ms

// 24 decoded bits (12 tri-state PT2262 symbols), 2 edges per bit
#define PT2262_BITS   24
#define MAX_EDGES    (PT2262_BITS * 2)

volatile uint32_t edgeBuf[MAX_EDGES];
volatile uint8_t  edgeCount  = 0;
volatile uint32_t lastEdge   = 0;
volatile bool     frameReady = false;
volatile uint32_t totalEdges = 0;   // diagnostic counter

void IRAM_ATTR gdo0ISR() {
  uint32_t now = micros();
  uint32_t dur = now - lastEdge;
  lastEdge = now;
  totalEdges++;

  if (dur > T_SYNC_MIN) {      // sync gap: start a new frame, discard this edge
    edgeCount  = 0;
    frameReady = false;
    return;
  }

  if (!frameReady && edgeCount < MAX_EDGES) {
    edgeBuf[edgeCount++] = dur;
    if (edgeCount == MAX_EDGES) frameReady = true;
  }
}

bool decodePT2262(uint32_t &code) {
  if (!frameReady) return false;

  noInterrupts();
  uint32_t t[MAX_EDGES];
  for (int i = 0; i < MAX_EDGES; i++) t[i] = edgeBuf[i];
  frameReady = false;
  edgeCount  = 0;
  interrupts();

#if PT2262_DEBUG
  Serial.print("Raw pulses [µs]: ");
  for (int i = 0; i < MAX_EDGES; i += 2) {
    Serial.printf("H%lu/L%lu ", t[i], t[i + 1]);
  }
  Serial.println();
#endif

  code = 0;
  for (int i = 0; i < PT2262_BITS; i++) {
    uint32_t hi = t[i * 2];
    uint32_t lo = t[i * 2 + 1];
    bool shortH = (hi >= T_SHORT_MIN && hi <= T_SHORT_MAX);
    bool longH  = (hi >= T_LONG_MIN  && hi <= T_LONG_MAX);
    bool shortL = (lo >= T_SHORT_MIN && lo <= T_SHORT_MAX);
    bool longL  = (lo >= T_LONG_MIN  && lo <= T_LONG_MAX);

    if      (shortH && longL)  code = (code << 1) | 0;  // bit "0"
    else if (longH  && shortL) code = (code << 1) | 1;  // bit "1"
    else if (shortH && shortL) code = (code << 1) | 0;  // float → 0
    else return false;                                   // malformed
  }
  return true;
}

void setup() {
  Serial.begin(115200);

  ELECHOUSE_cc1101.setSpiPin(12, 13, 11, 10);
  ELECHOUSE_cc1101.setGDO0(GDO0_PIN);

  if (ELECHOUSE_cc1101.getCC1101()) {
    Serial.println("CC1101 connection OK");
  } else {
    Serial.println("CC1101 connection Error — check wiring");
  }

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(CC1101_FREQ);
  ELECHOUSE_cc1101.setModulation(2);  // OOK/ASK — required for PT2262
  ELECHOUSE_cc1101.setDRate(3.79);    // ~350 µs pulse width
  ELECHOUSE_cc1101.setRxBW(200);      // 200 kHz RX bandwidth
  ELECHOUSE_cc1101.setSyncMode(0);    // no sync word — timing decoded manually
  ELECHOUSE_cc1101.setPktFormat(3);   // async: GDO0 = raw demodulated signal
  ELECHOUSE_cc1101.SetRx();

  pinMode(GDO0_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GDO0_PIN), gdo0ISR, CHANGE);

  Serial.println("Listening for PT2262 remote codes...");
}

void loop() {
  // Diagnostic: print edge count every 5 s so you can verify GDO0 is active
  static uint32_t diagMs = 0;
  if (millis() - diagMs >= 5000) {
    diagMs = millis();
    noInterrupts();
    uint32_t edges = totalEdges;
    totalEdges = 0;
    interrupts();
    Serial.printf("[diag] edges in last 5s: %lu  RSSI: %d dBm\n",
                  edges, ELECHOUSE_cc1101.getRssi());
  }

  uint32_t code = 0;
  if (decodePT2262(code)) {
    Serial.printf(">>> PT2262 code : 0x%06X (%lu)\n", code, code);
    Serial.printf("    Address     : 0x%05X\n", (code >> 4) & 0xFFFFF);
    Serial.printf("    Data        : 0x%X\n",   code & 0xF);
  }
}

