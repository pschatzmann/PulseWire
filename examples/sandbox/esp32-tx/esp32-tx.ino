// IR Transceiver Example for the ESP32 using a high performance RMT
// implementation


#include "Transceiver.h"
#include "Codecs.h"
#include "sandbox/DriverESP32.h"

const uint8_t txPin = 23;
const uint8_t frameSize = 20;
int carrierFreq = 0; 
uint32_t baud = 1000;

ManchesterPreamble preamble;
ManchesterCodec codec(preamble);
TxDriverESP32 tx(codec, txPin, carrierFreq);
Transceiver transceiver(tx);

void setup() {
  Serial.begin(115200);
  Logger::setLogLevel(Logger::LOG_LEVEL_INFO);
  transceiver.begin(baud);
}

void loop() {
  // Example: send a frame
  uint8_t data[frameSize] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100,
                              110, 120, 130, 140, 150, 160, 170, 180, 190, 200};
  if (tx.write(data, frameSize) == frameSize) {
    Serial.print("data sent:  ");                            
    for (int j=0;j<frameSize;j++){
      Serial.print("0x");
      Serial.print(data[j], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Send Error");
  } 
}
