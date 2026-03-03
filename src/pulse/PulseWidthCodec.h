#pragma once

#include <stddef.h>
#include <stdint.h>

#include "Codec.h"
#include "pulse/Vector.h"

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
 * Timing parameters are configurable.
 */
class PulseWidthCodec : public Codec {
 public:
  PulseWidthCodec() = default;

  PulseWidthCodec(Preamble& detector) : Codec(detector) {}

  CodecEnum getCodecType() const override { return CodecEnum::PulseWidth; }

  // Used by IRTransceiver to initialize codec with protocol-specific parameters
  virtual void init(Preamble& detector, uint32_t shortPulseUs = 600,
                    uint32_t longPulseUs = 1200, uint32_t toleranceUs = 200) {
    setPreamble(detector);
    _shortPulseUs = shortPulseUs;
    _longPulseUs = longPulseUs;
    _toleranceUs = toleranceUs;
  }

  bool begin(uint16_t bitFrequencyHz) override {
    Codec::begin(bitFrequencyHz);
    if (_shortPulseUs == 0) _shortPulseUs = 1000000UL / (bitFrequencyHz * 2);
    if (_longPulseUs == 0) _longPulseUs = 1000000UL / (bitFrequencyHz / 2);
    if (_toleranceUs == 0) _toleranceUs = _bitPeriodUs * 0.3;
    return true;
  }

  /**
   * @brief Fill output vector with PulseWidth OutputSpec(s) for a bit.
   *
   * @param bit The bit to encode (true/false).
   * @param bitPeriod The bit period in microseconds.
   * @param output Vector to append OutputSpec(s).
   * @return Number of OutputSpec entries added.
   */
  size_t encodeBit(bool bit, Vector<OutputEdge>& output) override {
    OutputEdge pulse, space;
    pulse.level = true;  // HIGH
    pulse.pulseUs = bit ? _longPulseUs : _shortPulseUs;
    space.level = false;            // LOW
    space.pulseUs = _shortPulseUs;  // Fixed space duration for both bits
    output.push_back(pulse);
    output.push_back(space);
    return 2;
  }

  size_t getBitCount() const override { return 16; }

  bool decodeByte(Vector<OutputEdge>& edges, uint8_t& result) const override {
    if (edges.size() < 16) return false;
    uint8_t byte = 0;
    for (int i = 0; i < 8; ++i) {
      if (bitMatch(edges[i * 2].pulseUs, true)) {
        byte |= (1 << (7 - i));
      } else if (bitMatch(edges[i * 2].pulseUs, false)) {
        // bit is 0
      } else {
        return false;
      }
    }
    result = byte;
    return true;
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
};

}  // namespace pulsewire
