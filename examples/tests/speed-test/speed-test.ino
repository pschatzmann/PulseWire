// This example tests the speed of the ManchesterCodec and drivers by sending
// frames at increasing baud rates on the same microcontroller: The tx and rx
// are connected to the same pins, so the data is transmitted and received on
// the same device. The baud rate is increased in each iteration of the loop,
// and the received data is printed to the serial monitor. This allows you to
// see how well the codec and drivers perform at different speeds.

#include "Codecs.h"
#include "DriverArduino.h"
#include "Transceiver.h"

const uint8_t rxPin = 22;
const uint8_t txPin = 23;
int baud = 100;
DigitalSignal digitalSignal;
ManchesterPreamble preamble;
ManchesterCodec codec(preamble);
RecorderCodec rec(codec);
TxDriverArduino tx(codec, txPin, digitalSignal);
RxDriverArduino rx(rec, rxPin);
Transceiver transceiver(rx, tx);

void setup() {
  Serial.begin(115200);
  delay(3000);
  Logger::setLogLevel(Logger::LOG_LEVEL_INFO);
}

void loop() {
  Serial.print("Speed test: baud=");
  Serial.println(baud);
  transceiver.begin(baud);


  // send a frame
  const uint8_t frameSize = 10;
  const uint8_t data[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A };  // Test data to encode

  size_t written = tx.write(data, frameSize);
  if (written != frameSize) {
    Serial.println("Error: Failed to write full frame");
  } else {
    Serial.print("Written: ");
    for (size_t i = 0; i < frameSize; ++i) {
      Serial.print((uint8_t)data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // receive a frame
  size_t n = 0;
  uint8_t buf[frameSize]{};
  if (rx.available() >= 0) {
    n = rx.readBytes(buf, frameSize);
  }

  Serial.print("written = ");
  Serial.print(written);
  Serial.print(" available = ");
  Serial.println(rx.available());

  // Print recorded edges
  Serial.print("Encoded Edges: ");
  Serial.println(rec.getRecordedEdges().size());
  for (const auto& edge : rec.getRecordedEdges()) {
    Serial.print("Level: ");
    Serial.print(edge.level);
    Serial.print(", Pulse: ");
    Serial.println(edge.pulseUs);
  }
  rec.clear();

  Serial.print("Received: ");
  for (size_t i = 0; i < n; ++i) {
    Serial.print((uint8_t)buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  transceiver.end();

  delay(1000);
}