#pragma once
#include <Arduino.h>

#include "TransceiverConfig.h"
#include "pulse/TxDriver.h"
#include "pulse/TxProtocol.h"
#include "pulse/codecs/Codec.h"
#include "pulse/tools/Logger.h"
#include "pulse/tools/RingBuffer.h"
#include "pulse/tools/Vector.h"

namespace pulsewire {

/**
 * @brief Provides common logic for transmitting signals using various framing
 * modes.
 *
 * The `TxDriverCommon` class is a foundational component for managing the
 * transmission of signals. It integrates a specified `TxProtocol` and `Codec`
 * implementation to handle the framing and buffering of data. The class is
 * responsible for:
 * - Managing the framing modes for data transmission.
 * - Buffering data dynamically and sending frames when the buffer is full.
 * - Encoding bytes into signal edges and timings using the provided `Codec`.
 * - Sending preambles, data, and end signals based on the selected framing
 * mode.
 *
 * This class supports dynamic configuration of frame sizes, framing modes, and
 * optional checksum appending for data integrity. It is designed to work with a
 * specified transmission pin and bit frequency.
 *
 * @note The `Codec` handles the encoding of bytes into signal edges and
 * timings, while the `TxProtocol` manages the protocol-specific details of the
 * transmission.
 */
class TxDriverCommon : public TxDriver {
 public:
  TxDriverCommon() = default;
  /**
   * @param codec IR codec
   * @param pin TX pin
   * @param signal Signal implementation
   * @param freqHz Bit frequency
   * @param useChecksum If true, append checksum to frame (default: false)
   */
  TxDriverCommon(TxProtocol& protocol, Codec& codec, uint8_t pin,

                 bool useChecksum = false) {
    init(protocol, codec, pin, useChecksum);
  }

  virtual void init(TxProtocol& protocol, Codec& codec, uint8_t pin,

                    bool useChecksum = false) {
    TRACE();
    _codec = &codec;
    _protocol = &protocol;
    _useChecksum = useChecksum;
    _pin = pin;
  }

  bool begin(uint32_t bitFrequencyHz) override {
    // make sure that frame size is set before beginning
    TRACE();
    _bitFrequencyHz = bitFrequencyHz;
    setFrameSize(_frameSize);
    _protocol->begin(bitFrequencyHz, _codec, _pin);
    return _codec->begin(bitFrequencyHz);
  }

  /// Set the expected frame size for dynamic data transmission.
  void setFrameSize(uint16_t size) {
    TRACE();
    Logger::debug("Setting frame size to %d", size);
    _frameSize = size;
    _byteBuffer.resize(size);
    _protocol->setFrameSize(size);
  }

  /// Set the framing mode for how data is sent (e.g., fixed-size frames,
  /// immediate write, or flush)
  void setFramingMode(FramingMode mode) override { _framingMode = mode; }

  int write(uint8_t byte) override {
    if (_byteBuffer.size() == 0) {
      return 0;  // No buffer space allocated
    }
    _byteBuffer.write(byte);
    if (_byteBuffer.isFull()) {
      // Handle full buffer (e.g., send frame)
      uint8_t frameData[_frameSize];
      for (int j = 0; j < _frameSize; j++) {
        frameData[j] = _byteBuffer.read();
      }
      sendPreamble();
      sendData(frameData, _frameSize);
      sendEnd();
    }
    return 1;
  }

  /// Build frames in the buffer and send when full
  size_t write(const uint8_t* data, size_t len) {
    Logger::debug("Writing %d bytes to TxDriverCommon", len);
    switch (_framingMode) {
      case FramingMode::FixedSize:
        for (size_t i = 0; i < len; i++) {
          write(data[i]);
        }
        break;
      case FramingMode::WriteBytes:
        for (size_t i = 0; i < len; i++) {
          write(data[i]);
        }
        flush();
        break;
    }
    return len;
  }

  void flush() override {
    if (_byteBuffer.available() == 0) {
      return;  // Nothing to flush
    }
    sendPreamble();
    uint8_t frameData[_frameSize]{};
    int len = _byteBuffer.readArray(frameData, _frameSize);
    sendData(frameData, len);
    sendEnd();
  }

 protected:
  Codec* _codec = nullptr;
  TxProtocol* _protocol = nullptr;
  RingBuffer<uint8_t> _byteBuffer;
  uint16_t _frameSize = DEFAULT_FRAME_SIZE;
  FramingMode _framingMode = FramingMode::WriteBytes;
  uint16_t _bitFrequencyHz = DEFAULT_BIT_FREQ_HZ;
  bool _useChecksum = false;
  uint8_t check_sum = 0;
  uint8_t _pin = -1;
  bool isPreambleSent = false;

  void sendPreamble() {
    Logger::debug("Sending preamble");
    assert(_protocol != nullptr);
    if (!isPreambleSent) _protocol->sendPreamble();
    isPreambleSent = true;
  }

  void sendData(const uint8_t* data, uint8_t len) {
    Logger::debug("Sending data: %d bytes", len);
    assert(_protocol != nullptr);
    _protocol->sendData(data, len);
  }

  void sendEnd() {
    Logger::debug("Sending end");
    assert(_protocol != nullptr);
    _protocol->sendEnd(_useChecksum);
    isPreambleSent = false;  // reset for next frame
  }
};  // end of TxDriverArduino

}  // end of namespace pulsewire
