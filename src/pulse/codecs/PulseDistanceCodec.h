#pragma once

#include <stddef.h>
#include <stdint.h>

#include "Codec.h"
#include "pulse/tools/Vector.h"

namespace pulsewire {

/**
 * @brief Pulse-distance encoding/decoding utility class for IR communication.
 *
 * Encodes bytes using pulse-distance modulation (e.g., NEC, RC5, custom
 * protocols). Decodes pulse-distance bitstreams to bytes, with frame and
 * checksum support.
 *
 * - Logical '1': long pulse, short space
 * - Logical '0': short pulse, long space
 * Timing parameters are configurable.
 */
class PulseDistanceCodec : public Codec {
 public:
  PulseDistanceCodec() = default;

  PulseDistanceCodec(Preamble& detector, uint32_t toleranceUs = 0,
                     uint32_t shortPulseUs = 0, uint32_t longPulseUs = 0)
      : Codec(detector),
        _shortPulseUs(shortPulseUs),
        _longPulseUs(longPulseUs),
        _toleranceUs(toleranceUs) {}

  // used by ir
  void init(Preamble& detector, uint32_t shortPulseUs = 0,
            uint32_t longPulseUs = 0, uint32_t toleranceUs = 0) {
    setPreamble(detector);
    _shortPulseUs = shortPulseUs;
    _longPulseUs = longPulseUs;
    _toleranceUs = toleranceUs;
  }

  bool begin(uint32_t bitFrequencyHz) override {
    Codec::begin(bitFrequencyHz);
    if (_shortPulseUs == 0) _shortPulseUs = 1000000UL / (bitFrequencyHz * 2);
    if (_longPulseUs == 0) _longPulseUs = 1000000UL / (bitFrequencyHz / 2);
    if (_toleranceUs == 0) _toleranceUs = 0.5 * _bitPeriodUs;

    _inFrame = _preamble->preambleLength() ==
               0;  // If preamble is defined, start in "not in frame" state

    return true;
  }

  size_t getEdgeCount() const override { return 16; }

  int getEndOfFrameDelayUs() override { return 2 * _longPulseUs; }

  CodecEnum getCodecType() const override { return CodecEnum::PulseDistance; }

  bool getIdleLevel() { return true; }

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
  /**
   * @brief Fill output vector with PulseDistance OutputSpec(s) for a bit.
   *
   * @param bit The bit to encode (true/false).
   * @param output Vector to append OutputSpec(s).
   * @return Number of OutputSpec entries added.
   */
  size_t encodeBit(bool bit, Vector<OutputEdge>& output) {
    OutputEdge pulse, space;
    pulse.level = false;
    pulse.pulseUs = bit ? _longPulseUs : _shortPulseUs;
    space.level = true;
    space.pulseUs = _shortPulseUs;

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
      // only consider low edges
      if (edge.level == false) {
        if (bitMatch(edge.pulseUs, true)) {
          byte |= (1 << (7 - bit));
        } else if (bitMatch(edge.pulseUs, false)) {
          // bit is 0
        } else {
          Logger::debug("Invalid pulse duration for bit %d: %d us", bit,
                        edge.pulseUs);
        }
        bit++;
      }
    }
    result = byte;
    return true;
  }

  bool bitMatch(uint32_t duration, bool bit) const {
    uint32_t expected = bit ? _longPulseUs : _shortPulseUs;
    bool rc = (duration >= expected - _toleranceUs &&
               duration <= expected + _toleranceUs);
    Logger::debug(
        "Bit match for bit %d: duration=%d, expected=%d, tolerance=%d, "
        "match=%s",
        bit ? 1 : 0, duration, expected, _toleranceUs, rc ? "YES" : "NO");
    return rc;
  }
};

}  // namespace pulsewire
