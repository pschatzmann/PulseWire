#pragma once

#include "Codecs.h"
#include "DriverArduino.h"
#include "Preambles.h"
#include "Transceiver.h"

namespace pulsewire {

/**
 * @brief IRTransceiver: High-level interface for sending and receiving IR
 * signals.
 *
 * This class manages the encoding, decoding, and transmission of IR signals
 * using supported protocols. It provides methods to configure protocol
 * parameters, handle input/output pins, and process IR data streams. The class
 * abstracts the details of protocol detection and signal timing, allowing users
 * to easily integrate IR communication into their applications.
 *
 * Features:
 * - Supports multiple IR protocols (NEC, Sony, RC5, etc.)
 * - Handles both transmission and reception of IR signals
 * - Configurable timing and protocol parameters
 * - Designed for use with Arduino-compatible hardware
 */
class IRTransceiver : public Stream {
 public:
  /// Constructor for defined Protocol
  IRTransceiver(IRProtocol& protocol, uint8_t rxPin, uint8_t txPin)
      : _irProtocol(&protocol),
        _rxPin(rxPin),
        _txPin(txPin),
        _signal(protocol.frequency()),
        _codec(protocol.codec()),
        _rx(_codec, _rxPin, protocol.frequency()),
        _tx(_codec, _txPin, _signal, protocol.frequency()),
        _transceiver(_rx, _tx) {
    _codec.init(protocol, protocol.shortPulseUs(), protocol.longPulseUs(),
                protocol.toleranceUs());
    setIRProtocol(protocol);  // Initialize with provided protocol info
  }

  /// Constructor for Multiple Protocols
  IRTransceiver(IRMultiProtocol& protocol, uint8_t rxPin, uint8_t txPin)
      : _irProtocol(&protocol),
        _rxPin(rxPin),
        _txPin(txPin),
        _signal(protocol.frequency()),
        _codec(protocol.codec()),
        _rx(_codec, _rxPin, protocol.frequency()),
        _tx(_codec, _txPin, _signal, protocol.frequency()),
        _transceiver(_rx, _tx) {
    _codec.init(protocol, protocol.shortPulseUs(), protocol.longPulseUs(),
                protocol.toleranceUs());
    // Initialize with provided protocol info
    setIRProtocol(protocol);
    // Register callback for protocol changes: detected by IRMultiProtocol and
    // handled in onChange()
    protocol.setCallback(onChange, this);
  }

  int available() override { return _transceiver.available(); }
  size_t readBytes(uint8_t* data, size_t len) {
    return _transceiver.readBytes(data, len);
  }
  int read() override { return _transceiver.read(); }
  int peek() override { return _transceiver.peek(); }
  void flush() override { _transceiver.flush(); }
  size_t write(uint8_t b) override { return _transceiver.write(b); }
  size_t write(const uint8_t* data, size_t len) {
    return _transceiver.write(data, len);
  }

  const IRProtocol& getInfo() const { return *_irProtocol; }

  // Allow updating protocol info and reconfiguring drivers on the fly
  void setIRProtocol(IRProtocol& info) {
    if (&info == _irProtocol) return;
    // Update codec and drivers with new protocol info
    _irProtocol = &info;
    // Multi Protocol Support
    //_irMultiProtocol.setActualProtocol(info);
    _codec.init(*_irProtocol, info.shortPulseUs(), info.longPulseUs(),
                info.toleranceUs());
    _rx.init(_codec, _rxPin, info.frequency());
    _tx.init(_txProtocol, _codec, _txPin, info.frequency());
    uint32_t baud = 1000000 / (info.shortPulseUs() + info.longPulseUs()) / 2;
    _txProtocol.begin(baud, &_codec, _txPin);
    _signal.setFrequency(info.frequency());
    _signal.setTxPin(_txPin);
    _transceiver.setFrameSize(info.dataLength());
    _transceiver.setFramingMode(
        FramingMode::FixedSize);  // No additional framing
    _transceiver.begin();
  }

  // Callback for protocol changes detected by IRMultiProtocol
  static void onChange(IRProtocolEnum proto, IRProtocol& info, void* ref) {
    IRTransceiver* transceiver = static_cast<IRTransceiver*>(ref);
    transceiver->setIRProtocol(info);
  }

  void begin() {
    setIRProtocol(*_irProtocol);  // Initialize with current protocol info
    _transceiver.begin();
  }

 private:
  IRProtocol* _irProtocol = nullptr;
  ToneSignal _signal;
  Codec& _codec;
  TxProtocolGeneric _txProtocol{&_signal};
  RxDriverArduino _rx;
  TxDriverArduino _tx;
  Transceiver _transceiver;
  uint8_t _rxPin;
  uint8_t _txPin;
};

/**
 * @brief Transmitter class for IR communication.
 *
 * The `IRTransmitter` class is a specialized version of the `IRTransceiver`
 * class designed for transmission-only operations. It provides a simplified
 * interface for sending IR signals using a TX driver.
 */

class IRTrasmitter : public IRTransceiver {
 public:
  IRTrasmitter(IRProtocol info, uint8_t txPin)
      : IRTransceiver(info, 255, txPin) {}
};

/**
 * @brief Receiver class for IR communication.
 *
 * The `IRReceiver` class is a specialized version of the `IRTransceiver` class
 * designed for reception-only operations. It provides a simplified interface
 * for receiving IR signals using an RX driver.
 */

class IRReceiver : public IRTransceiver {
 public:
  IRReceiver(IRProtocol info, uint8_t rxPin)
      : IRTransceiver(info, rxPin, 255) {}
  IRReceiver(uint8_t rxPin) : IRTransceiver(multi_protocol, rxPin, 255) {}

 protected:
  IRMultiProtocol multi_protocol;
};

}  // namespace pulsewire