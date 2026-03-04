#pragma once
#include "ManchesterCodec.h"

namespace pulsewire {

/**
 * @brief Differential Manchester encoding/decoding utility class for IR
 * communication.
 *
 * - Each bit period has a transition in the middle.
 * - Logical '0': transition at the start of the bit period.
 * - Logical '1': no transition at the start of the bit period.
 */
class DifferentialManchesterCodec : public ManchesterCodec {
 public:
  DifferentialManchesterCodec(Preamble& preambleDetector)
      : ManchesterCodec(preambleDetector) {}

  bool begin(uint16_t bitFrequencyHz) override {
    _lastLevelEncode = getIdleLevel();
    return ManchesterCodec::begin(bitFrequencyHz);
  }

  CodecEnum getCodecType() const override {
    return CodecEnum::DifferentialManchester;
  }

 protected:
  // Differential Manchester: each bit is two pulses, but encoding depends on
  // previous level
  size_t encodeBit(bool bit, Vector<OutputEdge>& output) override {
    OutputEdge first, second;

    if (bit) {
      // No transition at start, transition in middle
      first.level = _lastLevelEncode;
      first.pulseUs = _bitPeriodUs / 2;
      _lastLevelEncode = !_lastLevelEncode;
      second.level = _lastLevelEncode;
      second.pulseUs = _bitPeriodUs / 2;
    } else {
      // Transition at start, transition in middle
      _lastLevelEncode = !_lastLevelEncode;
      first.level = _lastLevelEncode;
      first.pulseUs = _bitPeriodUs / 2;
      _lastLevelEncode = !_lastLevelEncode;
      second.level = _lastLevelEncode;
      second.pulseUs = _bitPeriodUs / 2;
    }
    output.push_back(first);
    output.push_back(second);
    return 2;
  }

  bool decodeByte(Vector<OutputEdge>& edges, uint8_t& result) override {
    // Defensive: check edge count
    if (edges.size() != getEdgeCount()) return false;
    uint8_t& byte = result;
    byte = 0;

    // MSB first: bit 7 is first
    for (int i = 0; i < 8; ++i) {
      bool b0 = edges[i * 2].level;
      bool b1 = edges[i * 2 + 1].level;

      // Differential Manchester: transition at start = 0, no transition = 1
      bool transitionAtStart = (b0 != _lastLevelDecode);
      if (!transitionAtStart) {
        byte |= (1 << (7 - i));
      }
      // Update lastLevel for next bit (end of this bit period)
      _lastLevelDecode = b1;
    }
    // Update _lastLevel for next decode
    return true;
  }

  void reset() override {
    ManchesterCodec::reset();
    _lastLevelEncode = getIdleLevel();
    _lastLevelDecode = getIdleLevel();
  }

  bool getIdleLevel() const override {
    return false;  // Idle state is LOW
  }

 protected:
  bool _lastLevelEncode = false;
  bool _lastLevelDecode = false;
};

}  // namespace pulsewire