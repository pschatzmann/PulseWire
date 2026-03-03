#pragma once
#include <stdint.h>

namespace pulsewire {
/**
 * @brief Abstract base class for signal output.
 *
 * Provides an interface for sending a single bit (HIGH or LOW) to a hardware
 * pin or modulator.
 */
class SignalBase {
 public:
  virtual void setTxPin(uint8_t pin) = 0;
  virtual void sendBit(bool bit) = 0;

 protected:
  uint8_t _pin;
};
} // namespace pulsewire
