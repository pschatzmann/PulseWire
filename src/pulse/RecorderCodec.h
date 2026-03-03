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
    _recordedEdges.reserve(size * getBitCount());
  }

  bool decodeEdge(uint32_t durationUs, bool level, uint8_t& result) override {
    _recordedEdges.push_back({level, durationUs});
    return _ref.decodeEdge(durationUs, level, result);
  }
  bool begin(uint16_t bitFrequencyHz) {
    //_recordedEdges.clear();
    return _ref.begin(bitFrequencyHz);
  }

  CodecEnum getCodecType() const override { return _ref.getCodecType(); }

  virtual size_t getBitCount() const { return _ref.getBitCount(); }

  Vector<OutputEdge>& getRecordedEdges() { return _recordedEdges; }

  void clear() { _recordedEdges.clear(); }

  size_t encodeBit(bool bit, Vector<OutputEdge>& output) {
    return _ref.encodeBit(bit, output);
  };

  bool decodeByte(Vector<OutputEdge>& edges, uint8_t& result) const override {
    return _ref.decodeByte(edges, result);
  };

  void encodeByte(uint8_t byte, uint8_t* bits) const override {
    _ref.encodeByte(byte, bits);
  }

 protected:
  Codec& _ref;
  Vector<OutputEdge> _recordedEdges;

//   bool bitMatch(uint32_t duration, bool bit) const override {
//     return _ref.bitMatch(duration, bit);
//   }
};

}  // namespace pulsewire