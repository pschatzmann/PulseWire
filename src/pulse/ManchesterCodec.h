#pragma once
#include "Codec.h"
#include "SignalBase.h"
#include "assert.h"
#include "pulse/Vector.h"

namespace pulsewire {

/**
 * @brief Manchester encoding/decoding utility class for IR communication.
 *
 * The following IR protocols use Manchester encoding:
 * - Philips RC-5
 * - IRDA
 *
 * Provides instance methods for encoding bytes to Manchester bitstream and
 * decoding Manchester bitstream to bytes. Can be injected into drivers for
 * modularity and reuse.
 */
class ManchesterCodec : public Codec {
 public:

  ManchesterCodec() = default;
  
  ManchesterCodec(Preamble& preambleDetector) : Codec(preambleDetector) {}

  CodecEnum getCodecType() const override { return CodecEnum::Manchester; }

  bool begin(uint16_t bitFrequencyHz) override {
    // Set up default preamble if still using the built-in one
    if (_preamble == &_defaultPreamble) {
      _defaultPreamble.clear();
      uint32_t halfBit = (1000000UL / bitFrequencyHz) / 2;
      _defaultPreamble.addEdge(true, halfBit);  // single HIGH idle edge
    }
    return Codec::begin(bitFrequencyHz);
  }

  /**
   * @brief Decode incoming edges to reconstruct bytes. Handles multiple edges
   per bit for noise tolerance by averaging edge durations and applying
   decodeEdge() logic to the average. This allows for robust decoding even in
   noisy environments where multiple edges may be detected
   *
   * @param durationUs
   * @param level
   * @param result
   * @return true
   * @return false
   */
  bool decodeEdge(uint32_t durationUs, bool level, uint8_t& result) override {
    // // Filter idle gaps
    // if (durationUs > _bitPeriodUs * 2) {
    //   reset();
    //   return false;
    // }

    int edgePeriod = 0.45f * _bitPeriodUs;
    int edgeCount = durationUs / edgePeriod;
    if (edgeCount < 1) edgeCount = 1;
    if (edgeCount > 4) edgeCount = 4;
    int avg_period = durationUs / edgeCount;

    bool valid = false;
    for (int i = 0; i < edgeCount; ++i) {
      if (Codec::decodeEdge(avg_period, level, result)) {
        valid = true;
      }
    }
    return valid;
  }

  int getEndOfFrameDelayUs() override {
    return 16 * _bitPeriodUs; 
  }


 protected:

  /**
   * @brief Get the number of edges used to encode a byte (16 for Manchester).
   */
  size_t getEdgeCount() const override { return 16; }

  
  /**
   * @brief Fill output vector with Manchester OutputSpec(s) for a bit.
   *
   * @param bit The bit to encode (true/false).
   * @param _bitPeriodUs The bit period in microseconds.
   * @param output Vector to append OutputSpec(s).
   * @return Number of OutputSpec entries added.
   */
  size_t encodeBit(bool bit, Vector<OutputEdge>& output) override {
    // Manchester: each bit is two pulses
    OutputEdge first, second;
    if (bit) {
      first.level = true;  // HIGH
      first.pulseUs = _bitPeriodUs / 2;
      second.level = false;  // LOW
      second.pulseUs = _bitPeriodUs / 2;
    } else {
      first.level = false;  // LOW
      first.pulseUs = _bitPeriodUs / 2;
      second.level = true;  // HIGH
      second.pulseUs = _bitPeriodUs / 2;
    }
    output.push_back(first);
    output.push_back(second);
    return 2;
  }

  bool decodeByte(Vector<OutputEdge>& edges, uint8_t& result) override {
    assert(edges.size() == getEdgeCount());
    bool valid = true;

    uint8_t& byte = result;
    byte = 0;

    for (int i = 0; i < 8; ++i) {
      bool b0 = edges[i * 2].level;
      bool b1 = edges[i * 2 + 1].level;

      if (b0 == 1 && b1 == 0) {
        byte |= (1 << (7 - i));  // Set bit to 1
      } else if (b0 == 0 && b1 == 1) {
        // Bit is 0, do nothing
      } else {
        valid = false;  // Invalid Manchester pair
        Logger::error("Invalid Manchester pair at bit %d: b0=%d, b1=%d", i, b0,
                      b1);
        break;
      }
    }

    return valid;
  }
};

}  // namespace pulsewire
