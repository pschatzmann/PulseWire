#pragma once
#include "Codec.h"

namespace pulsewire {

/**
 * @brief NRZ (Non-Return-to-Zero) codec for serial-like encoding/decoding
 * with start/stop bit framing. 
 */
class NRZCodec : public Codec {
 public:
  NRZCodec(uint8_t stopBits = 1) : _stopBits(stopBits) {}

  NRZCodec(Preamble& preambleDetector, uint8_t stopBits = 0)
      : Codec(preambleDetector), _stopBits(stopBits) {}

  CodecEnum getCodecType() const override { return CodecEnum::NRZ; }

  size_t encode(uint8_t byte, Vector<OutputEdge>& output) override {
    size_t edgeCount = 0;

    // Start bit (LOW, since idle is HIGH)
    OutputEdge start;
    start.level = false;
    start.pulseUs = _bitPeriodUs;
    output.push_back(start);
    ++edgeCount;

    // Data bits (LSB first)
    for (int bit = 0; bit < 8; ++bit) {
      OutputEdge data;
      data.level = (byte & (1 << bit));
      data.pulseUs = _bitPeriodUs;
      output.push_back(data);
      ++edgeCount;
    }

    // Stop bits (high)
    for (uint8_t s = 0; s < _stopBits; ++s) {
      OutputEdge stop;
      stop.level = true;
      stop.pulseUs = _bitPeriodUs;
      output.push_back(stop);
      ++edgeCount;
    }

    return edgeCount;
  }

  bool decodeEdge(uint32_t durationUs, bool level, uint8_t& result) override {
    Logger::debug("[NRZCodec] decodeEdge: level=%s, duration=%d us",
                  level ? "HIGH" : "LOW ", durationUs);
    // Filter idle gaps
    if (level == getIdleLevel() && durationUs > getEndOfFrameDelayUs()) {
      Logger::debug("Idle gap detected: %d us, resetting decoder", durationUs);
      reset();
      return false;
    }

    // No preamble means every edge is part of a frame
    if (!_inFrame && _preamble->preambleLength() == 0) {
      _inFrame = true;
    }

    if (_inFrame) {
      if (durationUs < _bitPeriodUs * 1.5) {
        // Short pulse, likely a single bit
        return Codec::decodeEdge(durationUs, level, result);
      }
      // Split long pulses into individual bit-period edges
      int count = (durationUs + _bitPeriodUs / 2) / _bitPeriodUs;
      if (count < 1) count = 1;
      Logger::debug("Long pulse detected: %d us, splitting into %d edges",
                    durationUs, count);

      bool valid = false;
      for (int i = 0; i < count; ++i) {
        if (Codec::decodeEdge(_bitPeriodUs, level, result)) {
          valid = true;
        }
      }
      return valid;
    } else {
      return Codec::decodeEdge(durationUs, level, result);
    }
  }

  bool decodeByte(Vector<OutputEdge>& edges, uint8_t& result) override {
    bool valid = true;
    if (edges[0].level != false) {
      Logger::error("Invalid start bit: expected LOW, got HIGH: duration=%d us",
                    edges[0].pulseUs);
      return false;  // Start bit
    }

    uint8_t byte = 0;
    for (int i = 0; i < 8; ++i) {
      if (edges[i + 1].level) {
        byte |= (1 << i);
        Logger::debug("Bit %d: 1", i);
      } else {
        Logger::debug("Bit %d: 0", i);
      }
    }

    // Check stop bits
    for (int s = 0; s < _stopBits; ++s) {
      if (edges[9 + s].level != true) {
        Logger::error("Stop bit %d: 0, duration=%d us", s, edges[9 + s].pulseUs);
        //valid = false;
      } else {
        Logger::debug("Stop bit %d: 1, duration=%d us", s, edges[9 + s].pulseUs);
      }
    }
    result = byte;
    reset();  // Ready for next frame
    return valid;
  }

  void reset() override {
    Codec::reset();
    _decodeEdgeStream.clear();
  }

  size_t getEdgeCount() const override { return 1 + 8 + _stopBits; }

  int getEndOfFrameDelayUs() override {
    return (getEdgeCount() + 2) * _bitPeriodUs;
  }

  bool getIdleLevel() const override { return true; }

  void setStopBits(uint8_t stopBits) { _stopBits = stopBits; }

  uint8_t getStopBits() const { return _stopBits; }

 protected:
  uint8_t _stopBits;

  // Implementation of pure virtual function bitMatch
  bool bitMatch(uint32_t duration, bool bit) const {
    uint32_t expectedDuration = _bitPeriodUs;
    return (duration >= expectedDuration - (_bitPeriodUs / 2)) &&
           (duration <= expectedDuration + (_bitPeriodUs / 2));
  }
};

}  // namespace pulsewire
