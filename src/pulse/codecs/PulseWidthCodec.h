#pragma once

#include <stddef.h>
#include <stdint.h>

#include "Codec.h"
#include "pulse/tools/Vector.h"

namespace pulsewire {

/**
 * @brief Pulse-width encoding/decoding utility class for IR communication.
 *
 * Encodes bytes using pulse-width modulation (e.g., Sony SIRC, custom
 * protocols). Decodes pulse-width bitstreams to bytes, with frame and checksum
 * support.
 *
 * - Logical '1': long pulse
 * - Logical '0': short pulse
 *
 * Timing parameters are configurable.
 */
class PulseWidthCodec : public Codec {
 public:
  PulseWidthCodec() = default;

  PulseWidthCodec(Preamble& detector) : Codec(detector) {}

  // Used by IRTransceiver to initialize codec with protocol-specific parameters
  virtual void init(Preamble& detector, uint32_t shortPulseUs,
                    uint32_t longPulseUs, uint32_t toleranceUs) {
    setPreamble(detector);
    _shortPulseUs = shortPulseUs;
    _longPulseUs = longPulseUs;
    _toleranceUs = toleranceUs;
  }

  bool begin(uint32_t bitFrequencyHz) override {
    Codec::begin(bitFrequencyHz);
    if (_shortPulseUs == 0) _shortPulseUs = 1000000UL / (bitFrequencyHz * 2);
    if (_longPulseUs == 0) _longPulseUs = 1000000UL / (bitFrequencyHz / 2);
    if (_toleranceUs == 0) _toleranceUs = _bitPeriodUs * 0.3;

    _inFrame = false;

    return true;
  }

  CodecEnum getCodecType() const override { return CodecEnum::PulseWidth; }

  int getEndOfFrameDelayUs() override { return 2 * _longPulseUs; }

  size_t getEdgeCount() const override { return 16; }

  size_t encode(uint8_t byte, Vector<OutputEdge>& output) override {
    Vector<bool> bits;
    bits.reserve(8);
    encodeByte(byte, bits);
    size_t total = 0;
    for (int i = 0; i < 8; ++i) {
      total += encodeBit(bits[i], output);
    }
    return total;
  }

  bool decodeEdge(uint32_t durationUs, bool level, uint8_t& result) override {
    // ensure that edges are allocated
    assert(_decodeEdgeStream.capacity() > 0);

    // Filter idle gaps
    if (level == getIdleLevel() && durationUs > getEndOfFrameDelayUs()) {
      Logger::debug("Idle gap detected: %d us, resetting decoder", durationUs);
      reset();
      return false;
    }

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
    if (_decodeEdgeStream.size() == getEdgeCount()) {
      bool rc = decodeByte(_decodeEdgeStream, result);
      Logger::debug("Decoded byte: 0x%x", result);
      _decodeEdgeStream.clear();
      return rc;
    }
    return false;
  }

 protected:
  uint32_t _shortPulseUs = 0;
  uint32_t _longPulseUs = 0;
  uint32_t _toleranceUs = 0;

  bool bitMatch(uint32_t duration, bool bit) const {
    uint32_t expected = bit ? _longPulseUs : _shortPulseUs;
    return (duration >= expected - _toleranceUs &&
            duration <= expected + _toleranceUs);
  }

  /**
   * @brief Fill output vector with PulseWidth OutputSpec(s) for a bit.
   *
   * @param bit The bit to encode (true/false).
   * @param bitPeriod The bit period in microseconds.
   * @param output Vector to append OutputSpec(s).
   * @return Number of OutputSpec entries added.
   */
  size_t encodeBit(bool bit, Vector<OutputEdge>& output) {
    OutputEdge pulse, space;
    pulse.level = true;  // HIGH
    pulse.pulseUs = bit ? _longPulseUs : _shortPulseUs;
    space.level = false;            // LOW
    space.pulseUs = _shortPulseUs;  // Fixed space duration for both bits
    output.push_back(pulse);
    output.push_back(space);
    return 2;
  }
  bool decodeByte(Vector<OutputEdge>& edges, uint8_t& result) {
    if (edges.size() < 16) {
      Logger::error("Not enough edges to decode byte: %d", edges.size());
      return false;
    }
    uint8_t byte = 0;
    int bit = 0;
    for (auto& edge : edges) {
      // only consider high
      if (edge.level) {
        if (bitMatch(edge.pulseUs, true)) {
          byte |= (1 << (7 - bit));
        } else if (bitMatch(edge.pulseUs, false)) {
          // bit is 0
        } else {
          Logger::error("Invalid pulse duration for bit %d: %d us", bit,
                        edge.pulseUs);
        }
        bit++;
      }
    }
    result = byte;
    return true;
  }
};

}  // namespace pulsewire
