// This example tests the DifferentialManchesterCodec and drivers by sending
// frames at increasing baud rates on the same microcontroller:
//
// The tx and rx pins need to be connected together for this test, so the data
// is transmitted and received on the same device.

#include "Codecs.h"
#include "DriverArduino.h"
#include "Transceiver.h"

const uint8_t rxPin = 22;
const uint8_t txPin = 23;
int baud = 8000;
DigitalSignal digitalSignal;
ManchesterPreamble preamble;
DifferentialManchesterCodec codec(preamble);
TxDriverArduino tx(codec, txPin, digitalSignal);
RxDriverArduino rx(codec, rxPin);
Transceiver transceiver(rx, tx);

void setup() {
  Serial.begin(115200);
  Logger::setLogLevel(Logger::LOG_LEVEL_INFO);
  transceiver.begin(baud);
}

void loop() {
  // send a frame
  const uint8_t frameSize = 10;
  const uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05,
                          0x06, 0x07, 0x08, 0x09, 0x0A};  // Test data to encode

  size_t written = tx.write(data, frameSize);
  if (written != frameSize) {
    Serial.println("Error: Failed to write full frame");
  } else {
    Serial.print("Written:  ");
    for (size_t i = 0; i < frameSize; ++i) {
      Serial.print((uint8_t)data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // receive a frame
  size_t receivedCount = 0;
  uint8_t receiveBuffer[frameSize]{};

  receivedCount = rx.readBytes(receiveBuffer, frameSize);
  Serial.print("Received: ");
  for (size_t i = 0; i < receivedCount; ++i) {
    Serial.print((uint8_t)receiveBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // delay(1000);
}