// Binary Transmission Example for the generic (interrupt-based) implementation
// using direct digital writes to output bits on the TX pin.
// Connect the TX and RX pins together for testing.

#include "Transceiver.h"
#include "DriverArduino.h"

const uint8_t txPin = 5;
int baud = 1000;

DigitalSignal digitalSignal;
ManchesterPreamble preamble;
ManchesterCodec codec(preamble);
TxDriverArduino tx(codec, txPin, digitalSignal);
Transceiver transceiver(tx );

void setup() {
  Serial.begin(115200);
  transceiver.begin(baud);
}

void loop() {
  Serial.println("sending...");
  // Example: send a frame
  uint8_t data[] = {1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                             11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
  tx.write(data, sizeof(data));
  delay(1000);
}
