// Feather RP2040 CAN: bus test partner for the Nebulizer board.
// Sends heartbeat 0x456 every 1 s; echoes every received frame back with ID+1 (prints all traffic on USB serial).
#include <Adafruit_MCP2515.h>
Adafruit_MCP2515 mcp(PIN_CAN_CS);
static const long BITRATE = 500000;   // must match Nebulizer (default 500 kbps)
uint32_t hb = 0, lastTx = 0, rxN = 0, txN = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CAN_STANDBY, OUTPUT); digitalWrite(PIN_CAN_STANDBY, false);   // transceiver on
  pinMode(PIN_CAN_RESET, OUTPUT);   digitalWrite(PIN_CAN_RESET, true);   // nRESET: high = run
  uint32_t t = millis(); while (!Serial && millis() - t < 2000) delay(10);
  if (!mcp.begin(BITRATE)) { while (1) { Serial.println("MCP2515 init FAILED"); delay(1000); } }
  Serial.println("FEATHER CAN READY 500k");
}

void loop() {
  int n = mcp.parsePacket();
  if (n || mcp.packetId() != -1) {
    long id = mcp.packetId(); uint8_t d[8]; int len = 0;
    while (mcp.available() && len < 8) d[len++] = mcp.read();
    rxN++;
    Serial.printf("RX id=%lX len=%d:", id, len); for (int i = 0; i < len; i++) Serial.printf(" %02X", d[i]); Serial.println();
    mcp.beginPacket(id + 1); for (int i = 0; i < len; i++) mcp.write(d[i]); mcp.endPacket(); txN++;
  }
  if (millis() - lastTx >= 1000) {
    lastTx = millis(); hb++;
    mcp.beginPacket(0x456); mcp.write(hb >> 8); mcp.write(hb & 0xFF); mcp.endPacket(); txN++;
    Serial.printf("TX hb=%lu (rx %lu tx %lu)\n", hb, rxN, txN);
  }
}
