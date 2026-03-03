#include "IRTransceiver.h"

IRTrasmitter tx(IRInfoNEC, 3);

// The above is a simplified version for:
// const uint8_t txPin = 3;
// const uint16_t frameSize = IRInfoNEC.dataLength();
// ToneSignal toneSignal(IRInfoNEC.frequency());
// PulseWidthCodec codec(IRInfoNEC, IRInfoNEC.shortPulseUs(), IRInfoNEC.longPulseUs(), IRInfoNEC.toleranceUs());
// TxDriverArduino tx_driver(codec, txPin, toneSignal);
// Transceiver tx(tx_driver, FramingMode::FixedSize, frameSize);

void setup() {
  Serial.begin(9600);
  tx.begin();
}

void loop() {
  // Send a NEC code (0x20DF10EF is the code for "Power" on many remotes)
  uint8_t data[] = {0x20, 0xDF, 0x10, 0xEF};
  tx.write(data, sizeof(data));
  Serial.println("Sent NEC code: 0x20DF10EF");
  delay(5000); // Wait before sending again
}