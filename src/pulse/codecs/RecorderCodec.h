#pragma once
#include "Codec.h"

namespace pulsewire {

/**
 * @brief Codec Wrapper that records all edges passed to decodeEdge() for later
 * analysis or testing.
 *
 */
class RecorderCodec : public Codec {
 public:
  RecorderCodec(Codec& ref) : _ref(ref) {}

  void init(Preamble& detector, uint32_t shortPulseUs = 600,
            uint32_t longPulseUs = 1200, uint32_t toleranceUs = 200) override {
    _ref.init(detector, shortPulseUs, longPulseUs, toleranceUs);
  }
  void setFrameSize(uint16_t size) {
    _ref.setFrameSize(size);
    _recordedEdges.reserve(size * getEdgeCount());
  }

  bool begin(uint32_t bitFrequencyHz) {
    //_recordedEdges.clear();
    return _ref.begin(bitFrequencyHz);
  }

  CodecEnum getCodecType() const override { return _ref.getCodecType(); }

  size_t getEdgeCount() const { return _ref.getEdgeCount(); }

  int getEndOfFrameDelayUs() override { return _ref.getEndOfFrameDelayUs(); }

  Vector<OutputEdge>& getRecordedEdges() { return _recordedEdges; }

  void clear() { _recordedEdges.clear(); }

  size_t encode(uint8_t byte, Vector<OutputEdge>& output) override {
    return _ref.encode(byte, output);
  }

  bool decodeEdge(uint32_t durationUs, bool level, uint8_t& result) override {
    _recordedEdges.push_back({level, durationUs});
    return _ref.decodeEdge(durationUs, level, result);
  }

 protected:
  Codec& _ref;
  Vector<OutputEdge> _recordedEdges;
};

}  // namespace pulsewire