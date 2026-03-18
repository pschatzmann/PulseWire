//
// Example for ESP32 using RMT peripheral for transmission and reception
// - Uses TxProtocolESP32 and RxDriverESP32 for efficient IR communication
// - Demonstrates sending and receiving a frame of data with configurable baud rate

#include "Codecs.h"
#include "Transceiver.h"
#include "sandbox/DriverESP32.h"

const uint8_t txPin = 23;
const uint8_t rxPin = 22;
const uint8_t frameSize = 20;
int carrierFreq = 0;
uint32_t baud = 10000;

ManchesterPreamble preamble;
ManchesterCodec codec(preamble);
TxDriverESP32 tx(codec, txPin, carrierFreq);
RxDriverESP32 rx(codec, rxPin);
Transceiver transceiver(rx, tx);

void setup() {
  Serial.begin(115200);
  Logger::setLogLevel(Logger::LOG_LEVEL_INFO);
  transceiver.setFrameSize(40);
  transceiver.begin(baud);
}

void loop() {
  // send a frame
  uint8_t data[frameSize];
  for (int j = 0; j < frameSize; j++) data[j] = j;

  if (tx.write(data, frameSize) == frameSize) {
    Serial.print("Sent:     ");
    for (int j = 0; j < frameSize; j++) {
      Serial.print("0x");
      Serial.print(data[j], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Send Error");
  }

  // receive a frame
  if (rx.available() >= frameSize) {
    uint8_t buf[frameSize];
    size_t n = rx.readBytes(buf, frameSize);
    Serial.print("Received: ");
    for (size_t i = 0; i < n; ++i) {
      Serial.print("0x");
      Serial.print((uint8_t)buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}
