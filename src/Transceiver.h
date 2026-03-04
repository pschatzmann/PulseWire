#pragma once

#include <stddef.h>
#include <stdint.h>

#include "Stream.h"
#include "TransceiverConfig.h"
#include "pulse/tools/Logger.h"
#include "pulse/tools/RingBuffer.h"
#include "pulse/RxDriver.h"
#include "pulse/TxDriver.h"

namespace pulsewire {

/**
 * @brief Bi-directional IR transceiver class with Stream interface and driver
 * abstraction.
 *
 * This class provides a flexible, buffered interface for IR
 * communication. It supports pluggable RX and TX drivers, frame size
 * management, and integrates with the Arduino Stream API.
 */
class Transceiver : public Stream {
 public:
  /**
   * @brief Default constructor
   */
  Transceiver() = default;

  /**
   * @brief Constructor with RX and TX drivers
   */
  Transceiver(RxDriver& rx, TxDriver& tx) : _rxDriver(&rx), _txDriver(&tx) {}

  /**
   * @brief Constructor with RX driver
   */
  Transceiver(RxDriver& rx) : _rxDriver(&rx) {}

  /**
   * @brief Constructor with TX driver
   */
  Transceiver(TxDriver& tx) : _txDriver(&tx) {}

  /**
   * @brief Constructor with RX and TX drivers
   */
  Transceiver(RxDriver& rx, TxDriver& tx, FramingMode mode, size_t frameSize)
      : _rxDriver(&rx), _txDriver(&tx) {
    setFrameSize(frameSize);
    setFramingMode(mode);
  }

  /**
   * @brief Constructor with RX driver
   */
  Transceiver(RxDriver& rx, FramingMode mode, size_t frameSize)
      : _rxDriver(&rx) {
    setFrameSize(frameSize);
    setFramingMode(mode);
  }

  /**
   * @brief Constructor with TX driver
   */
  Transceiver(TxDriver& tx, FramingMode mode, size_t frameSize)
      : _txDriver(&tx) {
    setFrameSize(frameSize);
    setFramingMode(mode);
  }

  /**
   * @brief Set the RX driver
   */
  void setRxDriver(RxDriver* driver) { _rxDriver = driver; }

  /**
   * @brief Set the TX driver
   */
  void setTxDriver(TxDriver* driver) { _txDriver = driver; }

  /**
   * @brief Set the expected maximum frame size for RX/TX drivers and
   * single-char buffer
   */
  void setFrameSize(uint16_t size) {
    Logger::debug("Setting frame size to %d", size);
    _frameSize = size;
    _singleCharRxBuffer.resize(size);
    if (_rxDriver) {
      _rxDriver->setFrameSize(size);
    }
    if (_txDriver) {
      _txDriver->setFrameSize(size);
    }
  }

  bool setFramingMode(FramingMode mode) {
    if (_txDriver) {
      _txDriver->setFramingMode(mode);
      return true;
    }
    return false;
  }

  /**
   * @brief Initialize the transceiver and drivers
   */
  bool begin(uint16_t bitFrequencyHz = DEFAULT_BIT_FREQ_HZ) {
    TRACE();
    bool success = true;
    // intialize frame size if not set
    if (_frameSize == 0) setFrameSize(DEFAULT_FRAME_SIZE);

    if (_rxDriver) {
      success = _rxDriver->begin(bitFrequencyHz);
      if (!success) {
        Logger::error("Failed to initialize RX driver");
      }
    }
    if (_txDriver) {
      success = _txDriver->begin(bitFrequencyHz);
      if (!success) {
        Logger::error("Failed to initialize TX driver");
      }
    }
    return success;
    ;
  }

  /**
   * @brief End/stop the RX driver
   */
  void end() {
    if (_rxDriver) _rxDriver->end();
    if (_txDriver) _txDriver->end();
  }

  /**
   * @brief Write a single byte to the TX driver
   */
  size_t write(uint8_t byte) override { return _txDriver->write(byte); }

  /**
   * @brief Write a buffer to the TX driver
   */
  size_t write(const uint8_t* buffer, size_t size) override {
    if (_txDriver) {
      return _txDriver->write(buffer, size);
    }
    return 0;
  }
  /**
   * @brief Not implemented
   */
  void flush() override {
    if (_txDriver) {
      _txDriver->flush();
    }
  }

  /**
   * @brief Return the number of bytes available for writing (always 1024)
   */
  int availableForWrite() override { return 1024; }

  /**
   * @brief Return the number of bytes available to read
   */
  int available() override {
    return _singleCharRxBuffer.available() + _rxDriver->available();
  }

  /**
   * @brief Read a single byte from the buffer
   */
  int read() override {
    refillSingleCharBuffer();
    return _singleCharRxBuffer.read();
  }

  /**
   * @brief Read bytes into the provided buffer, up to the
   * specified length.
   *
   * @param buffer
   * @param length
   * @return size_t
   */

  virtual size_t readBytes(uint8_t* buffer, size_t length) {
    int c1 = _singleCharRxBuffer.readArray(buffer, length);
    int open = length - c1;
    int c2 = 0;
    if (open > 0) {
      c2 = _rxDriver->readBytes((uint8_t*)buffer + c1, open);
    }
    return c1 + c2;
  }

  /**
   * @brief Peek at the next byte in the buffer without removing it
   */
  int peek() override {
    refillSingleCharBuffer();
    return _singleCharRxBuffer.peek();
  }

 protected:
  RxDriver* _rxDriver = nullptr;
  TxDriver* _txDriver = nullptr;
  RingBuffer<uint8_t> _singleCharRxBuffer{64};
  int _frameSize = 0;

  void refillSingleCharBuffer() {
    if (_singleCharRxBuffer.isEmpty() && _rxDriver) {
      int len = _singleCharRxBuffer.size();
      uint8_t tmp[len];
      int read = _rxDriver->readBytes(tmp, len);
      _singleCharRxBuffer.writeArray(tmp, read);
    }
  }
};

/**
 * @brief Transmitter class for communication.
 *
 * The `Trasmitter` class is a specialized version of the `Transceiver` class
 * designed for transmission-only operations. It provides a simplified interface
 * for sending IR signals using a TX driver.
 */

class Trasmitter : public Transceiver {
 public:
  Trasmitter(TxDriver& tx) : Transceiver(tx) {}
};

/**
 * @brief Receiver class for communication.
 *
 * The `Receiver` class is a specialized version of the `Transceiver` class
 * designed for reception-only operations. It provides a simplified interface
 * for receiving IR signals using an RX driver.
 */

class Receiver : public Transceiver {
 public:
  Receiver(RxDriver& rx) : Transceiver(rx) {}
};

}  // namespace pulsewire
