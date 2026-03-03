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

  PulseDistanceCodec(Preamble& detector, uint32_t toleranceUs = 200,
                     uint32_t shortPulseUs = 600, uint32_t longPulseUs = 1200)
      : Codec(detector),
        _shortPulseUs(shortPulseUs),
        _longPulseUs(longPulseUs),
        _toleranceUs(toleranceUs) {}

  PulseDistanceCodec(Preamble& detector) : Codec(detector) {}

  CodecEnum getCodecType() const override { return CodecEnum::PulseDistance; }

  // used by ir
  void init(Preamble& detector, uint32_t shortPulseUs = 600,
            uint32_t longPulseUs = 1200, uint32_t toleranceUs = 200) {
    setPreamble(detector);
    _shortPulseUs = shortPulseUs;
    _longPulseUs = longPulseUs;
    _toleranceUs = toleranceUs;
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
    if (bit) {
      // Logical '1': long pulse, short space
      pulse.level = true;  // HIGH
      pulse.pulseUs = _longPulseUs;
      space.level = false;  // LOW
      space.pulseUs = _shortPulseUs;
    } else {
      // Logical '0': short pulse, long space
      pulse.level = true;  // HIGH
      pulse.pulseUs = _shortPulseUs;
      space.level = false;  // LOW
      space.pulseUs = _longPulseUs;
    }
    output.push_back(pulse);
    output.push_back(space);
    return 2;
  }

  size_t getBitCount() const override { return 16; }

  bool begin(uint16_t bitFrequencyHz) override {
    Codec::begin(bitFrequencyHz);
    if (_shortPulseUs == 0) _shortPulseUs = 1000000UL / (bitFrequencyHz * 2);
    if (_longPulseUs == 0) _longPulseUs = 1000000UL / (bitFrequencyHz / 2);
    if (_toleranceUs == 0) _toleranceUs = _bitPeriodUs * 0.3;
    return true;
  }

 protected:
  uint32_t _shortPulseUs = 0;
  uint32_t _longPulseUs = 0;
  uint32_t _toleranceUs = 0;

  bool decodeByte(Vector<OutputEdge>& edges, uint8_t& result) const override {
    assert(edges.size() == getBitCount());
    bool valid = true;
    uint8_t& byte = result;
    for (int i = 0; i < 8; ++i) {
      // For pulse-distance, each bit is represented by a pulse (HIGH) followed
      // by a space (LOW) We check the duration of the pulse to determine the
      // bit value
      if (bitMatch(edges[i * 2].pulseUs, true)) {
        byte |= (1 << (7 - i));  // Logical '1'
      } else if (bitMatch(edges[i * 2].pulseUs, false)) {
        // Logical '0', do nothing
      } else {
        // Invalid pulse, return 0 or set a valid flag if needed
        valid = false;
      }
    }
    return valid;
  }

  bool bitMatch(uint32_t duration, bool bit) const {
    uint32_t expected = bit ? _longPulseUs : _shortPulseUs;
    return (duration >= expected - _toleranceUs &&
            duration <= expected + _toleranceUs);
  }
};

}  // namespace pulsewire
