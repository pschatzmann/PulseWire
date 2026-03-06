// IR Transceiver Example for the ESP32 using a high performance RMT
// implementation

#include "Transceiver.h"
#include "sandbox/DriverESP32.h"

const uint8_t rxPin = 22;
const uint8_t frameSize = 20;
int baud = 1000;

ManchesterPreamble preamble;
ManchesterCodec codec(preamble);
RxDriverESP32 rx(codec, rxPin);
Transceiver transceiver(rx);

void setup() {
  Serial.begin(115200);
  transceiver.begin(baud);
}

void loop() {
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
}
