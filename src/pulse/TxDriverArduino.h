#pragma once
#include "TxDriverCommon.h"
#include "pulse/Logger.h"
#include "assert.h"

namespace pulsewire {

/**
 * @brief Generic driver for transmitting data using a specified codec and
 * signal implementation.
 *
 * The `TxDriverArduino` class extends the functionality of `TxDriverCommon` by
 * providing a generic implementation for transmitting data. It integrates a
 * `Codec` for encoding data and a `SignalBase` implementation for managing the
 * transmission signal.
 *
 * This class is responsible for:
 * - Initializing the transmission protocol with the provided signal
 * implementation.
 * - Managing the transmission pin and configuring the signal for data
 * transmission.
 * - Supporting dynamic configuration of transmission parameters such as
 * frequency and checksum usage.
 *
 * @note The `TxProtocolGeneric` is used to handle protocol-specific details,
 * while the `SignalBase` implementation manages the physical signal
 * transmission.
 */
class TxDriverArduino : public TxDriverCommon {
 public:
  TxDriverArduino() = default;
  TxDriverArduino(Codec& codec, uint8_t pin, SignalBase& signal,

                  bool useChecksum = false)
      : _txProtocol(&signal) {  // Initialize txProtocol with &signal
    this->_signal = &signal;
    init(_txProtocol, codec, pin, useChecksum);
  }

  void init(TxProtocol& protocol, Codec& codec, uint8_t pin,
             bool useChecksum = false) {
    TRACE();
    TxDriverCommon::init(protocol, codec, pin, useChecksum);
    assert(_signal != nullptr);
    _signal->setTxPin(pin);
    _signal->sendBit(codec.getIdleLevel());  // Ensure signal starts in idle state
  }

 protected:
  TxProtocolGeneric _txProtocol;
  SignalBase* _signal = nullptr;
};

}  // namespace pulsewire
