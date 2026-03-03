// IR Transceiver Example for the ESP32 using a high performance RMT
// implementation

#include "DriverESP32.h"
#include "Transceiver.h"

const uint8_t txPin = 17;
const uint8_t frameSize = 20;
int carrierFreq = 38000;
int baud = 500;

ManchesterPreamble preamble(baud);
ManchesterCodec codec(preamble);
TxDriverESP32 tx(codec, txPin, carrierFreq, baud);
Transceiver transceiver(tx);

void setup() {
  Serial.begin(115200);
  tx.begin(frameSize);
}

void loop() {
  // Example: send a frame
  uint8_t data[frameSize] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100,
                              110, 120, 130, 140, 150, 160, 170, 180, 190, 200};
  tx.write(data, frameSize);
  delay(1000);

}
