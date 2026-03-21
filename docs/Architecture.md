## Architecture

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
    - [Digital](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1DigitalSignal.html)
    - [Tone](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1ToneSignal.html) (modulated)
    - [PWM](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1PWMSignal.html) (modulated)

- [Codec](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1Codec.html)
    - [PulseDistance](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1PulseDistanceCodec.html)
    - [PulseWidth](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1PulseWidthCodec.html)
    - [Manchaster](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1ManchesterCodec.html)
    - [Differential Manchester](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1DifferentialManchesterCodec.html)
    - [NRZ](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1NRZCodec.html)
    - [RZ](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1RZCodec.html)
    - [Miller](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1MillerCodec.html)

- [Preamble](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1Preamble.html)
   - [NoPreamble](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1NoPreamble.html)
   - [CustomPreamble](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1CustomPreamble.html)
   - [CustomPreambleUs](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1CustomPreambleUs.html)
   - [ManchesterPreable](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1ManchesterPreamble.html)
   - [NRZPreamble](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1NRZPreamble.html)
   - [IRProtocol](https://pschatzmann.github.io/PulseWire/html/classpulsewire_1_1IRProtocol.html#details)

