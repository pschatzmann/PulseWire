#pragma once

#include <stddef.h>
#include <stdint.h>

#include "Codec.h"
#include "pulse/Vector.h"

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

  bool begin(uint16_t bitFrequencyHz) override {
    Codec::begin(bitFrequencyHz);
    if (_shortPulseUs == 0) _shortPulseUs = 1000000UL / (bitFrequencyHz * 2);
    if (_longPulseUs == 0) _longPulseUs = 1000000UL / (bitFrequencyHz / 2);
    if (_toleranceUs == 0) _toleranceUs = _bitPeriodUs * 0.4;

    _inFrame = false;

    return true;
  }

  /**
   * @brief Fill output vector with PulseDistance OutputSpec(s) for a bit.
   *
   * @param bit The bit to encode (true/false).
   * @param output Vector to append OutputSpec(s).
   * @return Number of OutputSpec entries added.
   */
  size_t encodeBit(bool bit, Vector<OutputEdge>& output) override {
    OutputEdge pulse, space;
    pulse.level = false;
    pulse.pulseUs = bit ? _longPulseUs : _shortPulseUs;
    space.level = true;
    space.pulseUs = _shortPulseUs;

    output.push_back(pulse);
    output.push_back(space);
    return 2;
  }

  size_t getEdgeCount() const override { return 16; }

  int getEndOfFrameDelayUs() override { return 2 * _longPulseUs; }

  CodecEnum getCodecType() const override { return CodecEnum::PulseDistance; }

  bool getIdleLevel() { return true; }

 protected:
  uint32_t _shortPulseUs = 0;
  uint32_t _longPulseUs = 0;
  uint32_t _toleranceUs = 0;

  bool decodeByte(Vector<OutputEdge>& edges, uint8_t& result) const override {
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
          Logger::error("Invalid pulse duration for bit %d: %d us", bit,
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
