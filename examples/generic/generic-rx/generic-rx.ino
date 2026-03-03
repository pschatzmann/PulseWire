// IR Transmission Example for the generic (interrupt-based) rx implementation
// and using the tone() function for modulation on the tx side.

#include "DriverArduino.h"
#include "Transceiver.h"
#include "Codecs.h"

const uint8_t rxPin = 4;
const uint8_t frameSize = 20;
int baud = 1000;

ManchesterPreamble preamble(baud);
ManchesterCodec codec(preamble);
RxDriverArduino rx(codec,rxPin);
Transceiver transceiver(rx);

void setup() {
  Serial.begin(115200);
  rx.begin(baud);
}

void loop() {
  // Example: send a frame
  uint8_t data[frameSize]{};
  // Example: receive a frame
  if (rx.available() >= frameSize) {
    uint8_t buf[frameSize];
    size_t n = rx.readBytes(buf, frameSize);
    Serial.print("Received: ");
    for (size_t i = 0; i < n; ++i) {
      Serial.print((uint8_t)buf[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
  delay(100);
}
