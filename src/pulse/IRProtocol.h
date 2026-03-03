#pragma once
#include "PulseDistanceCodec.h"
#include "PulseWidthCodec.h"
#include "ManchesterCodec.h"
#include "NRZCodec.h"
#include "Preamble.h"
#include "Codec.h"
#include "Vector.h"

namespace pulsewire {
/**
 * @brief Enum for supported IR protocols.
 */
enum class IRProtocolEnum {
  Unknown,
  Custom,
  NEC,
  NEC16,
  NEC42,
  Sony,
  RC5,
  RC6,
  Denon,
  JVC,
  Sharp,
  Kaseikyo,
  Samsung,
  Samsung48,
  Whynter,
  Manchester,
  Apple,
  Onkyo,
  Matsushita,
  Grundig,
  SiemensGigaset,
  Nokia,
  Thomson,
  Telefunken,
  Technics
};

/// convert IRProtocolEnum to string
const char* toStr(IRProtocolEnum protocol) {
  switch (protocol) {
    case IRProtocolEnum::Custom:
      return "Custom";
    case IRProtocolEnum::NEC:
      return "NEC";
    case IRProtocolEnum::NEC16:
      return "NEC16";
    case IRProtocolEnum::NEC42:
      return "NEC42";
    case IRProtocolEnum::Sony:
      return "Sony";
    case IRProtocolEnum::RC5:
      return "RC5";
    case IRProtocolEnum::RC6:
      return "RC6";
    case IRProtocolEnum::Denon:
      return "Denon";
    case IRProtocolEnum::JVC:
      return "JVC";
    case IRProtocolEnum::Sharp:
      return "Sharp";
    case IRProtocolEnum::Kaseikyo:
      return "Kaseikyo";
    case IRProtocolEnum::Samsung:
      return "Samsung";
    case IRProtocolEnum::Samsung48:
      return "Samsung48";
    case IRProtocolEnum::Whynter:
      return "Whynter";
    case IRProtocolEnum::Apple:
      return "Apple";
    case IRProtocolEnum::Onkyo:
      return "Onkyo";
    case IRProtocolEnum::Matsushita:
      return "Matsushita";
    case IRProtocolEnum::Grundig:
      return "Grundig";
    case IRProtocolEnum::SiemensGigaset:
      return "Siemens Gigaset";
    case IRProtocolEnum::Nokia:
      return "Nokia";
    case IRProtocolEnum::Thomson:
      return "Thomson";
    case IRProtocolEnum::Telefunken:
      return "Telefunken";
    case IRProtocolEnum::Technics:
      return "Technics";
    case IRProtocolEnum::Manchester:
      return "Manchester";
    default:
      return "Unknown";
  }
}

/**
 * @brief IRProtocol: Represents a specific IR protocol with all its parameters.
 *
 * Use one of the predefined static instances for common protocols:
 *
 * - IRProtocolNEC
 * - IRProtocolNEC16
 * - IRProtocolNEC42
 * - IRProtocolSony
 * - IRProtocolSamsung
 * - IRProtocolSamsung48
 * - IRProtocolWhynter
 * - IRProtocolApple
 * - IRProtocolOnkyo
 * - IRProtocolMatsushita
 * - IRProtocolGrundig
 * - IRProtocolSiemensGigaset
 * - IRProtocolNokia
 * - IRProtocolThomson
 * - IRProtocolTelefunken
 * - IRProtocolTechnics
 * - IRProtocolJVC
 * - IRProtocolSharp
 * - IRProtocolKaseikyo
 * - IRProtocolDenon
 */
class IRProtocol : public Preamble {
 public:
  IRProtocol() = default;
  IRProtocol(IRProtocolEnum proto, CodecEnum codecType, uint32_t frequency,
             size_t dataLength, uint32_t shortPulse, uint32_t longPulse,
             uint32_t tolerance, Vector<OutputEdge> edges)
      : _proto(proto),
        _frequency(frequency),
        _dataLength(dataLength),
        _shortPulseUs(shortPulse),
        _longPulseUs(longPulse),
        _toleranceUs(tolerance),
        _codecType(codecType),
        _edges(edges) {}

  // Destructor to clean up codec if allocated
  ~IRProtocol() {
    if (_codec != nullptr) {
      delete _codec;
      _codec = nullptr;
    }
  }

  /// Set all fields for this IR protocol, including preamble edges
  void begin(IRProtocolEnum proto, uint32_t frequency, size_t dataLength,
             uint32_t shortPulse, uint32_t longPulse, uint32_t tolerance,
             Vector<OutputEdge> edges,
             CodecEnum codecType = CodecEnum::PulseDistance) {
    _proto = proto;
    _frequency = frequency;
    _dataLength = dataLength;
    _shortPulseUs = shortPulse;
    _longPulseUs = longPulse;
    _toleranceUs = tolerance;
    _edges.clear();
    for (const auto& edge : edges) {
      _edges.push_back(edge);
    }
  }

  /// Copy all fields from another IRProtocol instance
  void copyFrom(const IRProtocol& other) {
    _proto = other._proto;
    _frequency = other._frequency;
    _dataLength = other._dataLength;
    _shortPulseUs = other._shortPulseUs;
    _longPulseUs = other._longPulseUs;
    _toleranceUs = other._toleranceUs;
    _codecType = other._codecType;
    _edges.clear();
    for (const auto& edge : other._edges) {
      _edges.push_back(edge);
    }
  }

  /// Returns the protocol enum identifier for this IR protocol
  virtual IRProtocolEnum getProtocolID() const { return _proto; }

  const char* name() const { return toStr(_proto); }

  /// Returns the preamble edges for this protocol
  virtual int getEdges(Vector<OutputEdge>& output) const override {
    output.clear();
    for (auto& edge : _edges) {
      output.push_back(edge);
    }
    return _edges.size();
  }
  /// Returns the number of edges in the preamble
  virtual size_t preambleLength() const override { return _edges.size(); }
  /// Returns the carrier frequency in Hz
  virtual uint32_t frequency() const { return _frequency; }
  /// Returns the expected data length in bytes (not including preamble)
  virtual size_t dataLength() const { return _dataLength; }
  /// Returns the duration of the short pulse in microseconds
  virtual uint32_t shortPulseUs() const { return _shortPulseUs; }
  /// Returns the duration of the long pulse in microseconds
  virtual uint32_t longPulseUs() const { return _longPulseUs; }
  /// Tolerance in microseconds for pulse duration matching during detection
  virtual uint32_t toleranceUs() const { return _toleranceUs; }
  /// Provides the codec associated with this protocol
  virtual Codec& codec() {
    if (_codec == nullptr) {
      switch (_codecType) {
        case CodecEnum::PulseDistance:
          _codec = new PulseDistanceCodec();
          break;
        case CodecEnum::PulseWidth:
          _codec = new PulseWidthCodec();
          break;
        case CodecEnum::Manchester:
          _codec = new ManchesterCodec();
          break;
        case CodecEnum::NRZ:
          _codec = new NRZCodec();
          break;
        default:
          _codec = new PulseWidthCodec();
          break;
      }
    }
    return *_codec;
  }

 private:
  IRProtocolEnum _proto = IRProtocolEnum::Unknown;
  Codec* _codec = nullptr;

  uint32_t _frequency = 38000;  // Default frequency in Hz
  size_t _dataLength = 4;
  // Add these fields for protocol-specific timings:
  uint32_t _shortPulseUs = 560;  // Duration of the short pulse in microseconds
  uint32_t _longPulseUs = 1690;  // Duration of the long pulse in microseconds
  uint32_t _toleranceUs = 200;   // Tolerance in microseconds
  std::vector<OutputEdge> _edges;
  CodecEnum _codecType;
};

static IRProtocol IRProtocolNEC(IRProtocolEnum::NEC, CodecEnum::PulseDistance,
                                38000, 32, 560, 1690, 200,
                                {{true, 9000}, {false, 4500}});
static IRProtocol IRProtocolNEC16(IRProtocolEnum::NEC16, CodecEnum::PulseDistance,
                                  38000, 16, 560, 1690, 200,
                                  {{true, 9000}, {false, 2250}});
static IRProtocol IRProtocolNEC42(IRProtocolEnum::NEC42, CodecEnum::PulseDistance,
                                  38000, 42, 560, 1690, 200,
                                  {{true, 9000}, {false, 4500}, {true, 562}, {false, 562}});
static IRProtocol IRProtocolSony(IRProtocolEnum::Sony, CodecEnum::PulseWidth,
                                 40000, 12, 600, 1200, 200,
                                 {{true, 2400}});
static IRProtocol IRProtocolSamsung(IRProtocolEnum::Samsung, CodecEnum::PulseDistance,
                                    38000, 32, 560, 1690, 200,
                                    {{true, 4500}, {false, 4500}});
static IRProtocol IRProtocolSamsung48(IRProtocolEnum::Samsung48, CodecEnum::PulseDistance,
                                      38000, 48, 560, 1690, 200,
                                      {{true, 4800}, {false, 4800}});
static IRProtocol IRProtocolWhynter(IRProtocolEnum::Whynter, CodecEnum::PulseDistance,
                                    38000, 32, 560, 1690, 200,
                                    {{true, 2850}, {false, 2850}});
static IRProtocol IRProtocolApple(IRProtocolEnum::Apple, CodecEnum::PulseDistance,
                                  38000, 32, 560, 1690, 200,
                                  {{true, 9000}, {false, 4500}});
static IRProtocol IRProtocolOnkyo(IRProtocolEnum::Onkyo, CodecEnum::PulseDistance,
                                  38000, 32, 560, 1690, 200,
                                  {{true, 9000}, {false, 4500}});
static IRProtocol IRProtocolMatsushita(IRProtocolEnum::Matsushita, CodecEnum::PulseDistance,
                                       37900, 48, 432, 1296, 200,
                                       {{true, 3456}, {false, 1728}});
static IRProtocol IRProtocolGrundig(IRProtocolEnum::Grundig, CodecEnum::Manchester,
                                    38000, 24, 560, 1690, 200,
                                    {{true, 2600}, {false, 600}});
static IRProtocol IRProtocolSiemensGigaset(IRProtocolEnum::SiemensGigaset, CodecEnum::Manchester,
                                           38000, 32, 560, 1690, 200,
                                           {{true, 9000}, {false, 4500}});
static IRProtocol IRProtocolNokia(IRProtocolEnum::Nokia, CodecEnum::Manchester,
                                  38000, 32, 560, 1690, 200,
                                  {{true, 4500}, {false, 4500}});
static IRProtocol IRProtocolThomson(IRProtocolEnum::Thomson, CodecEnum::Manchester,
                                    38000, 24, 560, 1690, 200,
                                    {{true, 2600}, {false, 600}});
static IRProtocol IRProtocolTelefunken(IRProtocolEnum::Telefunken, CodecEnum::Manchester,
                                       38000, 24, 560, 1690, 200,
                                       {{true, 2600}, {false, 600}});
static IRProtocol IRProtocolTechnics(IRProtocolEnum::Technics, CodecEnum::Manchester,
                                     37900, 48, 432, 1296, 200,
                                     {{true, 3456}, {false, 1728}});
static IRProtocol IRProtocolJVC(IRProtocolEnum::JVC, CodecEnum::PulseDistance,
                                38000, 32, 525, 1575, 200,
                                {{true, 8400}, {false, 4200}});
static IRProtocol IRProtocolSharp(IRProtocolEnum::Sharp, CodecEnum::PulseDistance,
                                  38000, 32, 320, 1600, 200,
                                  {{true, 3200}, {false, 1600}});
static IRProtocol IRProtocolKaseikyo(IRProtocolEnum::Kaseikyo, CodecEnum::PulseDistance,
                                     37900, 48, 432, 1296, 200,
                                     {{true, 3456}, {false, 1728}});
static IRProtocol IRProtocolDenon(IRProtocolEnum::Denon, CodecEnum::PulseDistance,
                                  38000, 24, 425, 1275, 200,
                                  {{true, 2600}, {false, 500}});

/**
 * @brief IRMultiProtocol: Handles detection and delegation for multiple IR
 * protocols.
 *
 * This class manages a collection of known IR protocol definitions and
 * attempts to detect which protocol is present in an incoming signal. It
 * delegates all IR protocol-specific method calls to a reference protocol
 * instance
 * (`_refInfo`), which is updated upon successful detection. This allows
 * seamless switching and handling of multiple IR protocols within a single
 * interface. The class also supports callback registration for protocol
 * detection events.
 */
class IRMultiProtocol : public IRProtocol {
 public:
  IRMultiProtocol() {
    _preambles.push_back(&IRProtocolNEC);
    _preambles.push_back(&IRProtocolNEC16);
    _preambles.push_back(&IRProtocolNEC42);
    _preambles.push_back(&IRProtocolSony);
    _preambles.push_back(&IRProtocolSamsung);
    _preambles.push_back(&IRProtocolSamsung48);
    _preambles.push_back(&IRProtocolWhynter);
    _preambles.push_back(&IRProtocolApple);
    _preambles.push_back(&IRProtocolOnkyo);
    _preambles.push_back(&IRProtocolMatsushita);
    _preambles.push_back(&IRProtocolGrundig);
    _preambles.push_back(&IRProtocolSiemensGigaset);
    _preambles.push_back(&IRProtocolNokia);
    _preambles.push_back(&IRProtocolThomson);
    _preambles.push_back(&IRProtocolTelefunken);
    _preambles.push_back(&IRProtocolTechnics);
    _preambles.push_back(&IRProtocolJVC);
    _preambles.push_back(&IRProtocolSharp);
    _preambles.push_back(&IRProtocolKaseikyo);
    _preambles.push_back(&IRProtocolDenon);
  }
  IRMultiProtocol(IRProtocol& defaultInfo) : IRMultiProtocol() {
    setActualProtocol(defaultInfo);
  }
  IRProtocolEnum getProtocolID() const override {
    return _refInfo->getProtocolID();
  }
  int getEdges(Vector<OutputEdge>& output) const override {
    return _refInfo->getEdges(output);
  }
  size_t preambleLength() const override { return _refInfo->preambleLength(); }
  uint32_t frequency() const { return _refInfo->frequency(); }
  size_t dataLength() const { return _refInfo->dataLength(); }
  uint32_t shortPulseUs() const { return _refInfo->shortPulseUs(); }
  uint32_t longPulseUs() const { return _refInfo->longPulseUs(); }
  uint32_t toleranceUs() const { return _refInfo->toleranceUs(); }

  // Optionally, delegate copyFrom and setup if needed
  void copyFrom(const IRProtocol& other) { _refInfo->copyFrom(other); }
  void begin(IRProtocolEnum proto, uint32_t frequency, size_t dataLength,
             uint32_t shortPulse, uint32_t longPulse, uint32_t tolerance,
             Vector<OutputEdge> edges) {
    _refInfo->begin(proto, frequency, dataLength, shortPulse, longPulse,
                    tolerance, edges);
  }

  bool detect(const OutputEdge& edge) override {
    for (size_t i = 0; i < _preambles.size(); ++i) {
      if (_preambles[i]->detect(edge)) {
        IRProtocolEnum _proto = _preambles[i]->getProtocolID();
        if (_callback) _callback(_proto, *_preambles[i], _ref);
        return true;
      }
    }
    return false;
  }

  /// You can registre a callback that will be executed when the protocol
  /// changes
  void setCallback(void (*callback)(IRProtocolEnum, IRProtocol&, void* ref),
                   void* ref) {
    _callback = callback;
    _ref = ref;
  }

  /// Provides the active protocol
  IRProtocol& getProtocol() const { return *_refInfo; }

  /// Provides a protocol by its enum ID, or a default if not found
  IRProtocol& getProtocolByID(IRProtocolEnum proto) const {
    for (size_t i = 0; i < _preambles.size(); ++i) {
      if (_preambles[i]->getProtocolID() == proto) {
        return *_preambles[i];
      }
    }
    static IRProtocol na;
    return na;  // Return default if not found
  }

  void addProtocol(IRProtocol& protocol) {
    // Check if protocol with same ID already exists
    for (size_t i = 0; i < _preambles.size(); ++i) {
      if (_preambles[i]->getProtocolID() == protocol.getProtocolID()) {
        _preambles[i] = &protocol;  // Replace existing protocol
        return;
      }
    }
    _preambles.push_back(&protocol);  // Add new protocol
  }

  void setActualProtocol(IRProtocol& proto) {
    _refInfo = &proto;
    if (_preambles[0] != &proto) {
      _preambles.insert(_preambles.begin(), _refInfo);
    }
  }

 protected:
  IRProtocol* _refInfo = &IRProtocolNEC;
  Vector<IRProtocol*> _preambles;
  void (*_callback)(IRProtocolEnum, IRProtocol&, void* ref) = nullptr;
  void* _ref = nullptr;
};

}  // namespace pulsewire