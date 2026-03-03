#pragma once
#include <Arduino.h>

#include "IRProtocol.h"

namespace pulsewire {

/**
 * @brief ProtocolDetector: Uses interrupt edge logic to detect IR protocol by
 * testing all known preambles.
 *
 * This class listens for IR edges and feeds them to all known preamble
 * detectors. When a preamble is detected, it returns the protocol as an enum.
 */
class IRProtocolDetector {
 public:
  IRProtocolDetector() {
    reset();
    _instance = this;
    _multiProtocol.setCallback(onProtocolDetected, this);
  }

  /// Starts protocol detection by attaching an interrupt to the specified pin.
  bool begin(uint8_t pin) {
    if (_pin != -1) end();  // Ensure any existing interrupt is detached
    _pin = pin;
    pinMode(_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_pin), interruptHandler, CHANGE);
    return true;
  }

  /// Stops protocol detection by detaching the interrupt.
  void end() {
    reset();
    detachInterrupt(digitalPinToInterrupt(_pin));
  }

  IRProtocolEnum getDetectedProtocol() const { return _detectedProtocol; }

  const char* getDetectedProtocolStr() const { return toStr(_detectedProtocol); } 

  void reset() { _detectedProtocol = IRProtocolEnum::Unknown; }

 protected:
  static IRProtocolDetector* _instance;
  uint8_t _pin = -1;
  IRMultiProtocol _multiProtocol;
  volatile IRProtocolEnum _detectedProtocol = IRProtocolEnum::Unknown;
  volatile uint32_t _lastEdge = 0;

  static void interruptHandler() {
    if (_instance) _instance->handleInterrupt();
  }

  static void onProtocolDetected(IRProtocolEnum proto, IRProtocol& info,
                                 void* ref) {
    IRProtocolDetector* detector = static_cast<IRProtocolDetector*>(ref);
    if (detector) detector->_detectedProtocol = proto;
  }


  void handleInterrupt() {
    bool level = digitalRead(_pin);
    uint32_t now = micros();
    uint32_t duration = now - _lastEdge;
    _lastEdge = now;
    OutputEdge edge{level, duration};
    _multiProtocol.detect(edge);
  }
};

// Static instance pointer definition
IRProtocolDetector* IRProtocolDetector::_instance = nullptr;

}  // namespace pulsewire
