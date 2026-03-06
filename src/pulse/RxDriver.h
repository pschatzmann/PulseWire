#pragma once
#include <stddef.h>
#include <stdint.h>

namespace pulsewire {

/**
 * @brief Abstract base class for IR receivers.
 */
class RxDriver {
 public:
  /// Set the expected frame size for dynamic data reception.
  virtual void setFrameSize(uint16_t size) = 0;
  /// Start the receiver.
  virtual bool begin(uint32_t bitFrequencyHz) = 0;
  /// Stop the receiver.
  virtual void end() = 0;
  /// Read up to 'size' bytes from the internal buffer into 'data'. Returns
  /// number of bytes read.
  virtual size_t readBytes(uint8_t* data, size_t size) = 0;
  /// Get the number of bytes available in the internal buffer.
  virtual int available() = 0;
  /// Set the size of the internal RX buffer.
  virtual void setRxBufferSize(size_t size) = 0;
};

}  // namespace pulsewire
