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

  NRZCodec(Preamble& preambleDetector, uint8_t stopBits = 1)
      : Codec(preambleDetector), _stopBits(stopBits) {}

  CodecEnum getCodecType() const override { return CodecEnum::NRZ; }

  bool getIdleLevel() { return true; }

  void setStopBits(uint8_t stopBits) { _stopBits = stopBits; }

  uint8_t getStopBits() const { return _stopBits; }

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
      data.level = (byte & (1 << bit)) != 0;
      data.pulseUs = _bitPeriodUs;
      output.push_back(data);
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

  bool decodeEdge(uint32_t durationUs, bool level, uint8_t& result) override {

    // Split long pulses into individual bit-period edges
    int count = (durationUs + _bitPeriodUs / 2) / _bitPeriodUs;
    if (count < 1) count = 1;
    if (count > (int)getEdgeCount()) count = getEdgeCount();

    bool valid = false;
    for (int i = 0; i < count; ++i) {
      if (Codec::decodeEdge(_bitPeriodUs, level, result)) {
        valid = true;
      }
    }
    return valid;
  }

  bool decodeByte(Vector<OutputEdge>& edges, uint8_t& result) override {
    if (edges.size() < 1 + 8 + _stopBits) return false;
    bool valid = true;
    if (edges[0].level != false) return false;  // Start bit

    uint8_t byte = 0;
    for (int i = 0; i < 8; ++i) {
      if (edges[i + 1].level) byte |= (1 << i);
    }

    // Check stop bits
    for (int s = 0; s < _stopBits; ++s) {
      if (edges[9 + s].level != true) valid = false;
    }

    result = byte;
    return valid;
  }

  size_t getEdgeCount() const override { return 1 + 8 + _stopBits; }

  int getEndOfFrameDelayUs() override { return getEdgeCount() + 1 * _bitPeriodUs;  }

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
