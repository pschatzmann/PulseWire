#pragma once
#include <Arduino.h>

#include "pulse/codecs/Codec.h"
#include "pulse/tools/RingBuffer.h"
#include "pulse/SignalBase.h"
#include "TransceiverConfig.h"
#include "pulse/TxDriver.h"
#include "pulse/tools/Logger.h"
#include "pulse/tools/Vector.h"

namespace pulsewire {

/**
 * @brief Digital output signal implementation using digitalWrite.
 *
 * Sends HIGH or LOW directly to the specified pin using digitalWrite().
 */
class DigitalSignal : public SignalBase {
 public:
  DigitalSignal() = default;

  void setTxPin(uint8_t pin) override {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
  }

  void sendBit(bool bit) {
    if (bit) {
      digitalWrite(_pin, HIGH);
    } else {
      digitalWrite(_pin, LOW);
    }
  }
};

/**
 * @brief Tone-modulated output signal implementation using tone().
 *
 * Sends a modulated carrier (using tone()) or disables it (using noTone()) for
 * IR transmission.
 */
class ToneSignal : public SignalBase {
 public:
  ToneSignal(uint16_t freq = CARRIER_HZ) : _freq(freq) {}

  void setFrequency(uint16_t freq) { _freq = freq; }
  uint16_t getFrequency() const { return _freq; }

  void setTxPin(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    noTone(_pin);
  }

  void sendBit(bool bit) {
    if (bit) {
      tone(_pin, _freq);
    } else {
      noTone(_pin);
    }
  }

 protected:
  uint16_t _freq;
};

#if defined(HAS_SET_PMW_FREQUENCY1) || defined(HAS_SET_PMW_FREQUENCY2)

/**
 * @brief PWM-modulated output signal implementation using analogWrite and
 * analogWriteFrequency.
 *
 * Sends a PWM carrier for IR transmission. HIGH enables PWM, LOW disables it.
 */
class PWMSignal : public SignalBase {
 public:
  PWMSignal(uint32_t freq = CARRIER_HZ, uint8_t duty = 128)
      : _freq(freq), _duty(duty) {}
  void setTxPin(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
#if defined(HAS_SET_PMW_FREQUENCY1)
    analogWriteFreq(_freq);
#endif
#if defined(HAS_SET_PMW_FREQUENCY2)
    analogWriteFrequency(_pin, _freq);
#endif
    analogWrite(_pin, 0);  // Start with output LOW
  }
  void sendBit(bool bit) override {
    if (bit) {
      analogWrite(_pin, _duty);  // Enable PWM
    } else {
      analogWrite(_pin, 0);  // Disable PWM (LOW)
    }
  }

 protected:
  uint32_t _freq;
  uint8_t _duty;
};

#endif

/**
 * @brief Abstract base class for defining transmission protocols.
 *
 * The `TxProtocol` class provides an interface for implementing various
 * transmission protocols. It defines the essential methods required for
 * managing the transmission of data, including preamble, data, and end signals.
 *
 * Responsibilities of the `TxProtocol` class include:
 * - Initializing the protocol with a codec and transmission pin.
 * - Configuring the size of the data to be transmitted.
 * - Sending preambles, data, and end signals based on the protocol's
 * requirements.
 * - Managing the state of the transmission frame.
 *
 * This class is designed to be extended by specific protocol implementations
 * that define the behavior of these methods.
 */

class TxProtocol {
 public:
  virtual bool begin(uint32_t bitFrequencyHz, Codec* p_codec, uint8_t pin) = 0;
  virtual void setFrameSize(uint16_t size) = 0;
  virtual void sendPreamble() = 0;
  virtual void sendData(const uint8_t* data, uint8_t len) = 0;
  virtual void sendEnd(bool& _useChecksum, bool isDelayAfterFrame) = 0;
  virtual bool isFrameClosed() const = 0;
};

/**
 * @brief Generic implementation that is supported by all Arduino cores
 *
 * The `TxProtocolGeneric` class provides a concrete implementation of the
 * `TxProtocol` interface, enabling the transmission of data using a specified
 * `SignalBase` implementation. It handles the encoding of data, preamble, and
 * checksum, and manages the state of the transmission frame.
 *
 * Responsibilities of the `TxProtocolGeneric` class include:
 * - Initializing the protocol with a codec and signal implementation.
 * - Encoding and transmitting preambles, data, and optional checksums.
 * - Managing the output buffer for storing encoded signal edges and timings.
 * - Ensuring proper frame closure and adding delays to signify the end of a
 * frame.
 *
 * This class is designed to work with a `SignalBase` implementation for
 * physical signal transmission and a `Codec` for encoding data into signal
 * edges and timings.
 */
class TxProtocolGeneric : public TxProtocol {
 public:
  /**
   * @brief Constructs a `TxProtocolGeneric` instance with the specified signal
   * implementation.
   * @param p_signal Pointer to the `SignalBase` implementation for signal
   * transmission.
   */
  TxProtocolGeneric(SignalBase* p_signal) { this->p_signal = p_signal; }

  /**
   * @brief Initializes the protocol with the specified codec and transmission
   * pin.
   * @param p_codec Pointer to the `Codec` used for encoding data.
   * @param pin Transmission pin for the signal.
   */
  bool begin(uint32_t bitFrequencyHz, Codec* p_codec, uint8_t pin) {
    TRACE();
    _bitFrequencyHz = bitFrequencyHz;
    _bitPeriod = 1000000UL / bitFrequencyHz;  // Bit period in microseconds
    this->_codec = p_codec;
    this->_codec->begin(bitFrequencyHz);
    return true;
  }

  /**
   * @brief Configures the size of the data to be transmitted.
   * @param size The size of the data in bytes.
   */
  void setFrameSize(uint16_t size) {
    TRACE();
    Logger::debug("Setting frame size to %d", size);
    _output_buffer.reserve(size);
  }

  /**
   * @brief Sends the preamble for the transmission.
   *
   * The preamble is encoded using the codec and transmitted using the signal
   * implementation. The checksum is reset at the start of a new frame.
   */
  void sendPreamble() {
    if (_codec == nullptr) {
      Logger::error("Codec pointer is null");
      _output_buffer.clear();
      return;
    }
    if (p_signal == nullptr) {
      Logger::error("SignalBase pointer is null");
      _output_buffer.clear();
      return;
    }
    if (is_frame_closed) {
      check_sum = 0;  // Reset checksum at the start of a new frame
      _output_buffer.clear();
      _codec->encodePreamble(_output_buffer);
      is_frame_closed = false;
    }
  }

  /**
   * @brief Encodes and transmits the data.
   * @param data Pointer to the data to be transmitted.
   * @param len Length of the data in bytes.
   * @param bitPeriod Duration of each bit in microseconds.
   */
  void sendData(const uint8_t* data, uint8_t len) {
    if (_codec == nullptr) {
      Logger::error("Codec pointer is null");
      _output_buffer.clear();
      return;
    }
    if (p_signal == nullptr) {
      Logger::error("SignalBase pointer is null");
      _output_buffer.clear();
      return;
    }
    for (uint8_t i = 0; i < len; i++) {
      check_sum += data[i];
      _codec->encode(data[i], _output_buffer);
    }
    // Flush any pending encoder state (e.g., Miller pending duration)
    _codec->flushEncoder(_output_buffer);
    for (const auto& spec : _output_buffer) {
      p_signal->sendBit(spec.level);
      delayUs(spec.pulseUs);
    }
  }

  /**
   * @brief Sends the end signal for the transmission.
   *
   * Optionally includes a checksum and adds a delay to signify the end of the
   * frame.
   * @param _useChecksum Whether to include the checksum in the transmission.
   * @param isDelayAfterFrame Whether to add a delay after the frame.
   * @param bitPeriod Duration of each bit in microseconds.
   */
  void sendEnd(bool& _useChecksum, bool isDelayAfterFrame) {
    if (_codec == nullptr) {
      Logger::error("Codec pointer is null");
      _output_buffer.clear();
      return;
    }
    if (p_signal == nullptr) {
      Logger::error("SignalBase pointer is null");
      _output_buffer.clear();
      return;
    }
    if (!is_frame_closed) {
      // print checksum
      if (_useChecksum) {
        _output_buffer.clear();
        _codec->encode(check_sum, _output_buffer);
        _codec->flushEncoder(_output_buffer);
        for (const auto& spec : _output_buffer) {
          p_signal->sendBit(spec.level);
          delayUs(spec.pulseUs);
        }
      }
      check_sum = 0;

      // Add a final delay to ensure we can recoginse the end of frame
      if (isDelayAfterFrame) {
        // Ensure signal is in idle state
        p_signal->sendBit(_codec->getIdleLevel());  
        uint32_t period =_codec->getEndOfFrameDelayUs();
        delayUs(period);
      }
      is_frame_closed = true;
    }
  }

  /**
   * @brief Checks whether the transmission frame is closed.
   * @return `true` if the frame is closed, `false` otherwise.
   */
  bool isFrameClosed() const { return is_frame_closed; }

 protected:
  uint8_t check_sum = 0;           ///< Checksum for data integrity.
  bool is_frame_closed = true;     ///< Indicates whether the frame is closed.
  SignalBase* p_signal = nullptr;  ///< Pointer to the signal implementation.
  Vector<OutputEdge>
      _output_buffer;       ///< Buffer for storing encoded signal edges.
  Codec* _codec = nullptr;  ///< Pointer to the codec used for encoding data.
  uint16_t _bitFrequencyHz = 0;
  uint16_t _bitPeriod = 0;

  inline void delayUs(uint32_t us) {
#if USE_RAW_DELAY
    delayMicroseconds(us);
#else
    uint32_t ms = us / 1000;
    uint32_t microSeconds = micros();
    delay(ms);
    int32_t elapsedUs = micros() - microSeconds;
    uint32_t remainingUs = (elapsedUs < us) ? (us - elapsedUs) : 0;
    if (remainingUs > 0) delayMicroseconds(remainingUs);
#endif
  }
};  // end of TxDriverArduino

}  // end of namespace pulsewire
