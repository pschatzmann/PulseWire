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

This library uses the Arduino Stream API, so that we can easily integrate with other Arduino functionality and provides quite a few __composable classes__, so that you can configure your individual communication scenario:

- Transceiver
  - RxDriver
    - Preamble
    - Codec
  - TxDriver
    - Preamble
    - Codec 
    - Signal 

- Signal 
    - Tone
    - Binary
    - PWM

- Codec
    - PulseDistance
    - PulseWidth
    - Manchaster
    - NRZ

- Preambles
   - None
   - Custom
   - NEC
   - Sony
   - Kaseikyo
   - Samsung
   - Whynter
   - ...



