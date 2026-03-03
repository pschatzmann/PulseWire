# PulseWire Arduino Communications Library

[![Arduino Library](https://img.shields.io/badge/Arduino-Library-blue.svg)](https://www.arduino.cc/reference/en/libraries/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

I did not find a library that could be used to transmit any arbitrary data with the help of an __Infrared Transmitter and Receiver__ module. Later I realized that this could be useful with any sceanrio where we want to transmit binary data __wirelessly or over a wire__ (e.g with  “433 MHz RF modules" or via ultrasound).

Data needs to be encoded into pulses to be transmitted and these pulses then need to be decoded on the receiver side: The most popular methods are __Manchaster Encoding__, __Pulse Distance__, __Pulse Width__ and __NRZ Encoding__. [Codecs](docs/Codecs.md) are providing this functionality.

On the receiver side, we need to be able to __recognise when a signal starts__. This can be done with a [Preamble](docs/Preambles) that descibes the starting pulses with their timings.

In addition, specific to transmitting IR data, we need to modulate the __signal__ with a carrier frequency (typically 38000 Hz). This can be done in different ways, so different alternaive implementations for this are provided.

## Key Features
- Modular Codec architecture: [Manchester, PulseDistance, PulseWidth or NRZ encoding](docs/Codecs.md).
- Flexible [transmittsion frequencies](docs/BitFrequency.md) when calling begin()
- Injectable, protocol-specific [preamble detection](docs/Preambles) and generation
- Arduino Stream API for easy integration with configurable [framing modes](docs/FramingModes.md) and frame size
- Protocol-agnostic drivers for generic Arduino 
- Hardware abstraction via OutputEdge for transmission and reception
- [Examples](examples) for all supported platforms and protocols
- An untested [draft implementation for IR Protocols](docs/IRProtocols.md)


## Architechture

This library uses the Arduino Stream API, so that we can easily integrate with other Arduino functionality and provides [quite a few __composable classes__](https://pschatzmann.github.io/PulseWire/html/annotated.html), so that you can configure your individual communication scenario:

- [Transceiver](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1Transceiver.html)
  - [RxDriver](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1RxDriver.html) ([RxDriverArduino](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1RxDriverArduino.html))
    - Preamble
    - Codec
  - [TxDriver](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1TxDriver.html) ([TxDriverArduino](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1TxDriverArduino.html))
    - Preamble
    - Codec 
    - Signal 

- [Signal](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1SignalBase.html) 
    - [Tone](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1ToneSignal.html)
    - [Digital](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1DigitalSignal.html)
    - [PWM](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1PWMSignal.html)

- [Codec](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1Codec.html)
    - [PulseDistance](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1PulseDistanceCodec.html)
    - [PulseWidth](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1PulseWidthCodec.html)
    - [Manchaster](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1ManchesterCodec.html)
    - [NRZ](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1NRZCodec.html)

- [Preamble](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1Preamble.html)
   - NoPreamble
   - CustomPreamble
   - CustomPreambleUs
   - ManchesterPreable
   - NRZPreamble
   - IRProtocol
      - Sony
      - Kaseikyo
      - Samsung
      - Whynter
      - ...


## Code Example

Here is a simple Arduino Sketch that sends binary manchester encoded data from one pin and receives it on another pin on the same microcontroller. Further exampes can be found in the [examples](examples) directory

```C++

#include "Codecs.h"
#include "DriverArduino.h"
#include "Transceiver.h"

const uint8_t rxPin = 22;
const uint8_t txPin = 23;
int baud = 8000;
DigitalSignal digitalSignal;
ManchesterPreamble preamble;
ManchesterCodec codec(preamble);
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
```