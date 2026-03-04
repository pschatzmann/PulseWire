#pragma once
#include "Codec.h"
namespace pulsewire {

/**
 * @brief Miller (Delay Modulation) encoding/decoding utility class.
 *
 * Miller encoding rules (bit period = T):
 * - '1': Transition in the MIDDLE of the bit period (T/2)
 * - '0' after '0': Transition at the START of the bit period
 * - '0' after '1': NO transition (level stays constant for full T)
 *
 * This implementation uses BYTE-ALIGNED encoding where each byte is
 * encoded independently. State is reset at byte boundaries to ensure
 * clean decoding.
 */
class MillerCodec : public Codec {
 public:
  MillerCodec() = default;
  MillerCodec(Preamble& preamble) : Codec(preamble) {}

  CodecEnum getCodecType() const override { return CodecEnum::Miller; }

  size_t getEdgeCount() const override { return 16; }

  int getEndOfFrameDelayUs() override { return 4 * _bitPeriodUs; }

  bool getIdleLevel() const override { return false; }

  void reset() override {
    Codec::reset();
    _encLevel = true;
    _rxByte = 0;
    _rxBitPos = 0;
    _rxExpectingMidTransition = false;
  }

  /**
   * @brief Encode a byte using Miller encoding (byte-aligned).
   *
   * Miller encoding rules:
   * - '1': Transition in the MIDDLE of the bit period (T/2)
   * - '0' after '0': Transition at the START of the bit period
   * - '0' after '1': NO transition (level stays constant for full T)
   *
   * Each byte is encoded independently with state reset at boundaries.
   * The encoder accumulates "pending" time at the current level and
   * outputs edges when transitions occur.
   */
  size_t encode(uint8_t byte, Vector<OutputEdge>& output) override {
    size_t count = 0;
    uint32_t halfT = _bitPeriodUs / 2;
    uint32_t pending =
        0;  // Accumulated time at current level before next transition
    bool prevBit =
        false;  // Track previous bit for Miller context (local to byte)

    Logger::debug("Miller encode: byte=0x%02X, startLevel=%d", byte, _encLevel);

    // Process each bit from MSB to LSB
    for (int i = 7; i >= 0; --i) {
      bool bit = (byte >> i) & 0x01;

      if (bit) {
        // '1' bit: Transition at middle of bit period
        // Add first half to pending, output edge, then start second half
        pending += halfT;
        OutputEdge edge;
        edge.level = _encLevel;
        edge.pulseUs = pending;
        output.push_back(edge);
        count++;
        _encLevel = !_encLevel;  // Toggle level after transition
        pending = halfT;         // Second half of '1' at new level
      } else {
        // '0' bit: Behavior depends on previous bit
        if (prevBit) {
          // '0' after '1': NO transition, just accumulate full bit period
          pending += _bitPeriodUs;
        } else {
          // '0' after '0' (or first bit): Transition at START of bit
          // Output any accumulated pending time, then start new accumulation
          if (pending > 0) {
            OutputEdge edge;
            edge.level = _encLevel;
            edge.pulseUs = pending;
            output.push_back(edge);
            count++;
            _encLevel = !_encLevel;
            pending = 0;
          }
          pending = _bitPeriodUs;  // This '0' bit's full period
        }
      }
      prevBit = bit;
    }

    // Flush any remaining pending time at end of byte
    if (pending > 0) {
      OutputEdge edge;
      edge.level = _encLevel;
      edge.pulseUs = pending;
      output.push_back(edge);
      count++;
      _encLevel = !_encLevel;
    }

    Logger::debug("Miller encode: produced %zu edges, endLevel=%d", count,
                  _encLevel);
    return count;
  }

  /**
   * @brief Add termination edge at end of frame.
   *
   * Adds a 2T pulse to ensure the receiver sees a final transition
   * and can complete decoding any pending bits before timeout.
   */
  size_t flushEncoder(Vector<OutputEdge>& output) override {
    size_t count = 0;

    // Add a 2T termination pulse to mark end of frame
    OutputEdge edge;
    edge.level = _encLevel;
    edge.pulseUs = 2 * _bitPeriodUs;  // 2T gives receiver time to process
    output.push_back(edge);
    count++;
    _encLevel = !_encLevel;

    Logger::debug("Miller flushEncoder: added termination edge (2T)");
    return count;
  }

  /**
   * @brief Decode Miller encoding edge by edge.
   *
   * Handles preamble detection using the base Codec's preamble logic,
   * then interprets edge durations in terms of half-periods (hp):
   * - hp=1 (0.5T): Half of a '1' bit (start or complete)
   * - hp=2 (1T):   One '0' bit with transition at boundary
   * - hp=3 (1.5T): '0' followed by start of '1', OR '1' followed by '0'
   * - hp=4 (2T):   Two '0' bits, OR '1' + '0' + start of '1'
   *
   * State machine tracks whether we're expecting the mid-transition of a '1'.
   */
  bool decodeEdge(uint32_t durationUs, bool level, uint8_t& result) override {
    // Handle idle gap (end of frame)
    if (handleIdleGap(durationUs, level, result)) {
      return true;
    }

    // Handle preamble detection
    if (!handlePreamble(durationUs, level)) {
      return false;
    }

    // Decode the Miller-encoded edge
    int hp = durationToHalfPeriods(durationUs);
    decodeMiller(hp, level);

    // Check if we've collected a complete byte
    return checkByteComplete(result);
  }

 protected:
  /**
   * @brief Handle idle gap detection and end-of-frame processing.
   * @return true if a complete byte was output, false otherwise.
   */
  bool handleIdleGap(uint32_t durationUs, bool level, uint8_t& result) {
    if (level == getIdleLevel() && durationUs > getEndOfFrameDelayUs()) {
      Logger::debug("Miller: Idle gap %lu us, resetting",
                    (unsigned long)durationUs);
      // If we were mid-'1', complete it before resetting
      if (_rxExpectingMidTransition && _rxBitPos > 0) {
        Logger::debug("  Completing pending '1' at end of frame");
        pushBit(true);
        if (_rxBitPos >= 8) {
          result = _rxByte;
          Logger::debug("  Byte complete at frame end: 0x%02X", result);
          reset();
          return true;
        }
      }
      reset();
    }
    return false;
  }

  /**
   * @brief Handle preamble detection.
   * @return true if we're in a frame and should process data, false to skip.
   */
  bool handlePreamble(uint32_t durationUs, bool level) {
    OutputEdge newEdge{level, durationUs};
    assert(_preamble != nullptr);
    if (!_inFrame) {
      if (_preamble->preambleLength() == 0) {
        // No preamble configured - first edge starts the frame
        // Don't process this edge as data - its duration is the idle gap
        _inFrame = true;
        Logger::debug("Miller: No preamble, starting new frame");
        return false;  // Skip this edge, wait for next
      } else if (_preamble->detect(newEdge)) {
        // Preamble detected - start new frame
        _inFrame = true;
        Logger::debug("Miller: Preamble detected, starting new frame");
        // Don't process the preamble edge as data, wait for next edge
        return false;
      } else {
        // Still waiting for preamble
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Convert duration to half-periods (rounded to nearest).
   */
  int durationToHalfPeriods(uint32_t durationUs) {
    uint32_t halfT = _bitPeriodUs / 2;
    int hp = (durationUs + halfT / 2) / halfT;
    if (hp < 1) hp = 1;
    if (hp > 4) hp = 4;  // Cap at 2T (4 half-periods)
    return hp;
  }

  /**
   * @brief Decode Miller half-periods into bits.
   */
  void decodeMiller(int hp, bool level) {
    Logger::debug("Miller decode: hp=%d, bitPos=%d, expectMid=%d", hp,
                  _rxBitPos, _rxExpectingMidTransition);

    if (_rxExpectingMidTransition) {
      decodeWhileExpectingMid(hp);
    } else {
      decodeAtBoundary(hp);
    }
  }

  /**
   * @brief Decode when expecting mid-transition of a '1' bit.
   */
  void decodeWhileExpectingMid(int hp) {
    if (hp == 1) {
      // 0.5T: Second half of '1' - complete the bit
      Logger::debug("  '1' complete");
      pushBit(true);
      _rxExpectingMidTransition = false;
    } else if (hp == 2) {
      // 1T = 0.5T + 0.5T: Complete '1', then start another '1'
      Logger::debug("  '1', start '1'");
      pushBit(true);
      // Stay in expectMid state for next '1'
    } else if (hp == 3) {
      // 1.5T = 0.5T + 1T: Complete '1', then full '0' bit
      Logger::debug("  '1', '0'");
      pushBit(true);
      pushBit(false);
      _rxExpectingMidTransition = false;
    } else if (hp == 4) {
      // 2T = 0.5T + 1T + 0.5T: Complete '1', '0', start '1'
      Logger::debug("  '1', '0', start '1'");
      pushBit(true);
      pushBit(false);
      // Stay in expectMid state for next '1'
    }
  }

  /**
   * @brief Decode when at bit boundary (not expecting mid-transition).
   */
  void decodeAtBoundary(int hp) {
    if (hp == 1) {
      // 0.5T: First half of '1' bit
      Logger::debug("  start '1'");
      _rxExpectingMidTransition = true;
    } else if (hp == 2) {
      // 1T: One '0' bit (with transition at boundary)
      Logger::debug("  '0'");
      pushBit(false);
    } else if (hp == 3) {
      // 1.5T = 1T + 0.5T: '0' bit, then start of '1'
      Logger::debug("  '0', start '1'");
      pushBit(false);
      _rxExpectingMidTransition = true;
    } else if (hp == 4) {
      // 2T: Two '0' bits
      Logger::debug("  '0', '0'");
      pushBit(false);
      pushBit(false);
    }
  }

  /**
   * @brief Check if a complete byte has been assembled.
   * @return true if byte complete, with result set.
   */
  bool checkByteComplete(uint8_t& result) {
    if (_rxBitPos >= 8) {
      result = _rxByte;
      Logger::debug("  Byte complete: 0x%02X", result);
      _rxByte = 0;
      _rxBitPos = 0;
      return true;
    }
    return false;
  }

  /**
   * @brief Push a decoded bit into the receive buffer.
   *
   * Shifts the bit into _rxByte from the right (MSB first order).
   * Ignores bits if byte is already complete (overflow protection).
   */
  void pushBit(bool bit) {
    if (_rxBitPos >= 8) {
      Logger::debug("  WARNING: bit overflow, ignoring bit %d", bit);
      return;
    }
    _rxByte = (_rxByte << 1) | (bit ? 1 : 0);
    _rxBitPos++;
    Logger::debug("    Pushed bit %d, pos=%d, byte=0x%02X", bit, _rxBitPos,
                  _rxByte);
  }

  // Encoder state
  bool _encLevel = true;  // Current output level (starts HIGH for visibility)

  // Decoder state
  uint8_t _rxByte = 0;    // Byte being assembled from received bits
  uint8_t _rxBitPos = 0;  // Number of bits received (0-8)
  bool _rxExpectingMidTransition = false;  // True if in first half of '1' bit
};

}  // namespace pulsewire
