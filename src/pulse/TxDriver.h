#pragma once
#include <stddef.h>
#include <stdint.h>
namespace pulsewire {

/**
 * @brief Framing modes for data transmission.
 */
enum class FramingMode {
  FixedSize,   // Send when buffer is full
  WriteBytes,  // Send immediately with writeBytes
  Flush        // Send all buffered data on flush
};

/**
 * @brief Abstract base class for IR transmitters.
 */
class TxDriver {
 public:
  virtual void setFrameSize(uint16_t size) = 0;
  virtual void setFramingMode(FramingMode mode) = 0;
  virtual int write(uint8_t byte) = 0;
  virtual size_t write(const uint8_t* data, size_t len) = 0;
  virtual bool begin(uint32_t bitFrequencyHz) = 0;
  virtual void flush() {};
  virtual void end() {};
};

}  // namespace pulsewire
