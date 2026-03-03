#pragma once
#include <stddef.h>
#include <stdint.h>
#include <assert.h>

#include "Preamble.h"
#include "SignalBase.h"
#include "pulse/Vector.h"

namespace pulsewire {

/// List of supported codec types
enum class CodecEnum { PulseDistance, PulseWidth, Manchester, NRZ };

const char* toStr(CodecEnum codec) {
  switch (codec) {
    case CodecEnum::PulseDistance:
      return "PulseDistance";
    case CodecEnum::PulseWidth:
      return "PulseWidth";
    case CodecEnum::Manchester:
      return "Manchester";
    case CodecEnum::NRZ:
      return "NRZ";
    default:
      return "Unknown";
  }
}

/**
 * @brief Abstract base class for IR protocol encoding and decoding.
 *
 * Provides a unified interface for encoding bytes to protocol-specific
 * bitstreams, edge-based decoding for protocol-agnostic drivers, and protocol
 * symbol count. Derived classes implement protocol-specific logic (e.g.,
 * Manchester, pulse-distance).
 */
class Codec {
 public:
  Codec() = default;

  Codec(Preamble& preambleDetector) : _preamble(&preambleDetector) {}

  /// Used by IRTransceiver to initialize codec with protocol-specific
  /// parameters
  virtual void init(Preamble& detector, uint32_t shortPulseUs = 600,
                    uint32_t longPulseUs = 1200, uint32_t toleranceUs = 200) {}

  /**
   * @brief  initialization method for codecs that require setup before
   * use (e.g., loading PIO programs, configuring state machines).
   */
  virtual bool begin(uint16_t bitFrequencyHz) {
    TRACE();
    assert(_preamble != nullptr);
    _inFrame = false;
    _bitFrequencyHz = bitFrequencyHz;
    _bitPeriodUs = 1000000UL / bitFrequencyHz;  // Bit period in microseconds
    _preamble->begin(bitFrequencyHz);
    _decodeEdgeStream.reserve(getBitCount());
    return true;
  }

  virtual bool getIdleLevel() {
    return false;
  }

  void reset() {
    _decodeEdgeStream.clear();  // clear() keeps capacity, no realloc
    _inFrame = false;
    _preamble->reset();
  }

  /**
   * @brief Get the preamble detector associated with this codec.
   * @return Reference to the Preamble instance used for preamble detection.
   */
  Preamble& getPreamble() { return *_preamble; }

  /**
   * @brief Set the Preamble Detector object
   *
   * @param preamble
   */
  void setPreamble(Preamble& preamble) { _preamble = &preamble; }
  /**
   * @brief Get the number of protocol symbols (bits, pulses, etc.) per encoded
   * byte.
   * @return Number of protocol symbols per byte.
   */
  virtual size_t getBitCount() const = 0;

  /**
   * @brief Edge-based decoding for protocol-agnostic RX drivers.
   *
   * Called on each signal edge (duration since last edge, new level, minUs,
   * maxUs). The codec maintains its own state and assembles frames internally.
   *
   * @param durationUs Time in microseconds since last edge.
   * @param level New logic level after the edge (true = HIGH, false = LOW).
   * @param frameBuffer Output buffer for decoded frame (if available).
   * @param frameBufferSize Size of frameBuffer (bytes).
   * @param frameLen Set to decoded frame length if a frame is available.
   * @return True if a complete frame is decoded and available in frameBuffer.
   */

  virtual bool decodeEdge(uint32_t durationUs, bool level, uint8_t& result) {
    // ensure that edges are allocated
    assert(_decodeEdgeStream.capacity() > 0);

    OutputEdge newEdge{level, durationUs};
    assert(_preamble != nullptr);
    if (!_inFrame) {
      if (_preamble->preambleLength() == 0) {
        // no preamble, the edge is valid playload
        _decodeEdgeStream.clear();
        _decodeEdgeStream.push_back(newEdge);
        _inFrame = true;
        Logger::debug("No preamble, starting new frame");
      } else if (_preamble->detect(newEdge)) {
        // Detected preamble, start new frame
        _inFrame = true;
        _decodeEdgeStream.clear();
        Logger::debug("Preamble detected, starting new frame");
      }
    } else {
      _decodeEdgeStream.push_back(newEdge);
    }

    // Try to decode bytes if enough edges collected
    if (_decodeEdgeStream.size() == getBitCount()) {
      bool rc = decodeByte(_decodeEdgeStream, result);
      Logger::debug("Decoded byte: 0x%x", result);
      _decodeEdgeStream.clear();
      return rc;
    }
    return false;
  }

  size_t encodePreamble(Vector<OutputEdge>& output) {
    if (_preamble == nullptr) return 0;
    return _preamble->getEdges(output);
  }

  /**
   * @brief Fill output vector with protocol-specific OutputSpec(s) for a
   * byte.
   *
   * Encodes the byte to protocol bits and appends OutputSpec(s) for each
   * bit.
   *
   * @param byte The byte to encode.
   * @param output Vector to append OutputSpec(s).
   * @return Number of OutputSpec entries added.
   */
  virtual size_t encode(uint8_t byte, Vector<OutputEdge>& output) {
    int count = 8;
    uint8_t bits[count];
    encodeByte(byte, bits);
    size_t total = 0;
    for (int i = 0; i < count; ++i) {
      total += encodeBit(bits[i], output);
    }
    return total;
  }

  /// Get the codec type (e.g., PulseDistance, Manchester) for this Codec
  /// instance.
  virtual CodecEnum getCodecType() const = 0;

  /// @brief  Get the name of the codec type as a string (e.g., "PulseDistance",
  /// "Manchester").
  const char* name() const { return toStr(getCodecType()); }

  void setFrameSize(uint16_t size) {
    _decodeEdgeStream.reserve(size * getBitCount());
  }

  /**
   * @brief Fill output vector with protocol-specific OutputSpec(s) for a bit.
   *
   * @param bit The bit to encode (true/false).
   * @param output Vector to append OutputSpec(s).
   * @return Number of OutputSpec entries added.
   */
  virtual size_t encodeBit(bool bit, Vector<OutputEdge>& output) { return 0; };

  /**
   * @brief Encode a byte to protocol bitstream. Default implementation encodes
   * to raw bits (MSB first), but can be overridden by protocols that require
   * different bit formatting (e.g., Manchester). The output bits are stored in
   *
   * @param byte The input byte to encode.
   * @param bits Output buffer for encoded bits (protocol-specific format).
   */
  virtual void encodeByte(uint8_t byte, uint8_t* bits) const {
    for (int i = 7; i >= 0; --i) {
      bool bit = (byte >> i) & 0x01;
      bits[7 - i] = bit ? 1 : 0;
    }
  }

  virtual bool decodeByte(Vector<OutputEdge>& edges, uint8_t& result) const = 0;

 protected:
  CustomPreambleUs _defaultPreamble;
  Preamble* _preamble = &_defaultPreamble;
  uint16_t _bitFrequencyHz = 0;
  uint32_t _bitPeriodUs = 0;
  Vector<OutputEdge> _decodeEdgeStream;
  volatile bool _inFrame = false;
};

}  // namespace pulsewire