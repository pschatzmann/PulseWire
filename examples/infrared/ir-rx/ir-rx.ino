#include "IRTransceiver.h"

IRReceiver rx(IRInfoNEC, 3); // Use NEC protocol on pin 3

void setup() {
  Serial.begin(9600);
  rx.begin();
}

void loop() {
  if (rx.available()) {
    uint8_t buffer[64];
    size_t len = rx.readBytes(buffer, sizeof(buffer));
    Serial.print("Received NEC code: 0x");
    for (size_t i = 0; i < len; i++) {
      Serial.print(buffer[i], HEX);
    }
    Serial.println();
  }
}   

