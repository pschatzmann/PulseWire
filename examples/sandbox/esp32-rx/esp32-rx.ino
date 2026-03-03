// IR Transceiver Example for the ESP32 using a high performance RMT
// implementation

#include "DriverESP32.h"
#include "Transceiver.h"

const uint8_t rxPin = 16;
const uint8_t frameSize = 20;
int carrierFreq = 38000;
int baud = 500;

ManchesterPreamble preamble(baud);
ManchesterCodec codec(preamble);
RxDriverESP32 rx(codec, rxPin, baud);
Transceiver transceiver(rx);

void setup() {
  Serial.begin(115200);
  rx.begin(frameSize);
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
  delay(100);
}
