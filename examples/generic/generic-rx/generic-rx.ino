// Generic manchester data receive example for the (interrupt-based) rx
// implementation.

#include "Codecs.h"
#include "DriverArduino.h"
#include "Transceiver.h"

const uint8_t rxPin = 4;
int baud = 1000;

ManchesterPreamble preamble(baud);
ManchesterCodec codec(preamble);
RxDriverArduino rx(codec, rxPin);
Transceiver transceiver(rx);

void setup() {
  Serial.begin(115200);
  rx.begin(baud);
}

void loop() {
  // Example: receive
  uint8_t data[1024]{};
  size_t n = rx.readBytes(data, 1024);
  if (n > 0) {
    Serial.print("Received: ");
    for (size_t i = 0; i < n; ++i) {
      Serial.print((uint8_t)data[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}
