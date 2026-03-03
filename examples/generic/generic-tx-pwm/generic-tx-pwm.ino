// IR transmission example for the generic (interrupt-based) manchester rx implementation
// and pwm modulation on the tx side.

#include "DriverArduino.h"
#include "Transceiver.h"

const uint8_t txPin = 5;
const uint8_t frameSize = 20;
int carrierFreq = 38000;
int baud = 1000;

PWMSignal pwm(carrierFreq);
ManchesterPreamble preamble;
ManchesterCodec codec(preamble);
TxDriverArduino tx(codec, txPin, pwm);
Transceiver transceiver(tx);


void setup() {
  Serial.begin(115200);
  transceiver.begin(baud);
}

void loop() {
  // Example: send a frame
  uint8_t data[frameSize] = {1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                             11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
  tx.write(data, frameSize);
  delay(1000);
}
