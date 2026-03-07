#pragma once
#include "Codec.h"
#include "pulse/tools/RingBuffer.h"

namespace pulsewire {

/**
 * Return-to-Zero (RZ) Codec implementation.
 * Each bit is represented by a pulse (high) for half the bit period, then zero
 * (low) for the other half.
 */
class RZCodec : public Codec {
 public:
  RZCodec(int stopBits = 1) : _stopBits(stopBits) {}

  RZCodec(Preamble& preambleDetector, int stopBits = 1) : Codec(preambleDetector), _stopBits(stopBits) {}

  virtual ~RZCodec() = default;

   bool begin(uint32_t bitFrequencyHz) override {
    if (_preamble == &_defaultPreamble) {
      _defaultPreamble.clear();
      // setup default preamble: alternating edges at full bit period, ending
      // with a HIGH level to match NRZ idle state
      uint32_t bp = 1000000UL / bitFrequencyHz;
      _defaultPreamble.addEdge(false, bp);  // single HIGH idle edge
      _defaultPreamble.addEdge(true, bp);   // single HIGH idle edge
    }

    return Codec::begin(bitFrequencyHz);
  }
 
  size_t encode(uint8_t byte, Vector<OutputEdge>& output) override {
    size_t edgeCount = 0;

    // Start bit (LOW)
    OutputEdge start;
    start.level = false;
    start.pulseUs = _bitPeriodUs;
    output.push_back(start);
    ++edgeCount;

    // Data bits (LSB first)
    for (int bit = 0; bit < 8; ++bit) {
      OutputEdge data;
      if (byte & (1 << bit)) {
        data.level = true;
        data.pulseUs = _bitPeriodUs / 2;
        output.push_back(data);
        data.level = false;
        data.pulseUs = _bitPeriodUs / 2;
        output.push_back(data);

      } else {
        data.level = false;
        data.pulseUs = _bitPeriodUs;
        output.push_back(data);
      }
      ++edgeCount;
    }

    // Stop bits (HIGH)
    for (uint8_t s = 0; s < _stopBits; ++s) {
      OutputEdge stop;
      stop.level = true;
      stop.pulseUs = _bitPeriodUs;
      output.push_back(stop);
      ++edgeCount;
    }

    return edgeCount;
  }

  // Decode a single edge (durationUs, level) and accumulate bits, implementing
  virtual bool decodeEdge(uint32_t durationUs, bool level,
                          uint8_t& result) override {
    _edgeBuffer.clear();
    Logger::debug("[RZCodec] decodeEdge: level=%s, duration=%d us",
                  level ? "HIGH" : "LOW ", durationUs);
    // Filter idle gaps
    if (level == getIdleLevel() && durationUs > getEndOfFrameDelayUs()) {
      reset();
      return false;
    }

    // Extracted frame start logic
    bool frameStarted = handleFrameStart(level, durationUs);

    // If frame just started, do NOT process this edge as data; wait for next
    // edge
    if (frameStarted) {
      return false;
    }
    // If already in frame, process as usual
    if (_inFrame) {
      return handleInFrameDecoding(durationUs, level, result);
    }
    return false;
  }

  CodecEnum getCodecType() const override { return CodecEnum::RZ; }

  bool getIdleLevel() const override { return true; }

  size_t getEdgeCount() const override { return 16; }

  int getEndOfFrameDelayUs() override { return 8 * _bitPeriodUs; }

  void reset() override {
    Codec::reset();
    _bitCount = 0;
    _byte = 0;
    _rxState = WAIT_START;
    _rxDataBitCount = 0;
    _rxStopBitCount = 0;
    _edgeBuffer.clear();
  }

 private:
  uint8_t _stopBits = 1;
  uint8_t _bitCount = 0;
  uint8_t _byte = 0;
  RingBuffer<OutputEdge> _edgeBuffer{32};
  enum { WAIT_START, DATA_BITS, STOP_BITS } _rxState = WAIT_START;
  uint8_t _rxDataBitCount = 0;
  uint8_t _rxStopBitCount = 0;

  // Helper to check if a pulse duration is within tolerance of a target period
  bool isValidPeriod(uint32_t pulse, uint32_t target,
                     uint32_t tolerance) const {
    return abs((int)pulse - (int)target) < (int)tolerance;
  }

  // Handle preamble and frame start logic
  bool handleFrameStart(bool level, uint32_t durationUs) {
    OutputEdge newEdge{level, durationUs};
    if (!_inFrame) {
      if (_preamble->preambleLength() == 0) {
        _decodeEdgeStream.clear();
        _decodeEdgeStream.push_back(newEdge);
        _inFrame = true;
        Logger::debug("No preamble, starting new frame");
        return true;
      } else if (_preamble->detect(newEdge)) {
        _inFrame = true;
        _decodeEdgeStream.clear();
        Logger::debug("Preamble detected, starting new frame");
        return true;
      }
    }
    return false;
  }

  // Handle in-frame bit/byte decoding logic
  bool handleInFrameDecoding(uint32_t durationUs, bool level, uint8_t& result) {
    // Reconstruct edges as before
    if (level) {
      _edgeBuffer.write({level, durationUs});
      Logger::debug("Reconstructed edge: level=%s, duration=%d us",
                    level ? "HIGH" : "LOW", durationUs);
    } else if (!level && durationUs >= _bitPeriodUs) {
      int n = durationUs / _bitPeriodUs;
      Logger::debug("Reconstructed %d edges: level=LOW, duration=%d us", n,
                    durationUs);
      for (int i = 0; i < n; ++i) {
        _edgeBuffer.write({false, _bitPeriodUs});
      }
    } else {
      Logger::debug("Ignored edge: level=LOW, duration=%d us", durationUs);
    }

    // State machine for start, data, and stop bits
    result = 0;
    while (!_edgeBuffer.isEmpty()) {
      OutputEdge edge;
      _edgeBuffer.read(edge);
      switch (_rxState) {
        case WAIT_START:
          // Expect start bit (LOW)
          if (!edge.level) {
            Logger::debug("Detected start bit");
            _byte = 0;
            _rxDataBitCount = 0;
            _rxStopBitCount = 0;
            _rxState = DATA_BITS;
          } else {
            Logger::warning("Expected start bit (LOW), got HIGH, skipping");
          }
          break;
        case DATA_BITS:
          // Data bits (LSB first)
          if (edge.level) {
            _byte |= (1 << _rxDataBitCount);
            Logger::debug("Decoded data bit 1 at pos=%d", _rxDataBitCount);
          } else {
            Logger::debug("Decoded data bit 0 at pos=%d", _rxDataBitCount);
          }
          if (++_rxDataBitCount == 8) {
            _rxState = STOP_BITS;
            _rxStopBitCount = 0;
          }
          break;
        case STOP_BITS:
          // Expect stop bits (HIGH)
          if (edge.level) {
            Logger::debug("Detected stop bit %d", _rxStopBitCount + 1);
          } else {
            Logger::warning("Expected stop bit (HIGH), got LOW, skipping");
          }
          if (++_rxStopBitCount >= _stopBits) {
            Logger::debug("Decoded byte: 0x%02x", _byte);
            result = _byte;
            _rxState = WAIT_START;
            return true;
          }
          break;
      }
    }
    return false;
  }
};

}  // namespace pulsewire
