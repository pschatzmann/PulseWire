#pragma once
#include <stdint.h>

#include "TransceiverConfig.h"
#include "pulse/tools/Logger.h"
#include "pulse/tools/Vector.h"

namespace pulsewire {

/**
 * @struct OutputEdge
 * @brief Specifies a single IR signal segment for protocol-agnostic
 * transmission.
 *
 * Used by Codec classes to describe the logic level and duration of each
 * segment (pulse or space) for hardware drivers. This enables protocol-agnostic
 * encoding and transmission, allowing drivers to convert OutputSpec vectors to
 * hardware-specific formats (e.g., RMT symbols, PWM, etc.).
 *
 * - level: Logic level during the segment (true = HIGH, false = LOW)
 * - pulseUs: Duration of the segment in microseconds
 */
struct OutputEdge {
  bool level =
      false;  // Logic level during the pulse (true = HIGH, false = LOW)
  uint32_t pulseUs = 0;  // Duration of the pulse in microseconds
  OutputEdge() = default;
  OutputEdge(bool l, uint32_t p) : level(l), pulseUs(p) {}
};

/**
 * @brief Abstract base class for preamble detection and generation.
 *
 * Provides an interface for detecting protocol-specific preambles in incoming
 * pulse streams and generating preamble edges for transmission. This allows
 * Codec classes to use different preamble detection strategies without coupling
 * to specific implementations.
 */

class Preamble {
 public:
  virtual ~Preamble() = default;

  virtual bool begin(uint32_t bitFrequencyHz) {
    _history.reserve(preambleLength());
    return true;
  }

  void reset() { _history.clear(); }

  /// Detects if the incoming edge matches the expected preamble pattern.
  virtual bool detect(const OutputEdge& edge) {
    Logger::debug("Preamble Detecting edge: level=%s, pulseUs=%d", edge.level ? "HIGH" : "LOW",
                  edge.pulseUs);
    size_t N = preambleLength();
    if (N == 0) return true;
    _history.push_back(edge);
    if (_history.size() > N) _history.erase(_history.begin());
    if (_history.size() < N) {
      Logger::debug("Not enough edges for preamble detection: %d/%d",
                    _history.size(), N);
      return false;
    }

    for (size_t i = 0; i < N; ++i) {
      if (_history[i].level != _expected[i].level) {
        Logger::debug("Invalid level idx %d: expected %s, got %s - %d us", i,
                      _expected[i].level ? "HIGH" : "LOW",
                      _history[i].level ? "HIGH" : "LOW   ",
                      _history[i].pulseUs);
        return false;
      }
      // Use 20% tolerance or 50us minimum
      uint32_t tol = 0.2 * _expected[i].pulseUs;
      if (tol < 50) tol = 50;
      if (!inRange(_history[i].pulseUs, _expected[i].pulseUs, tol)) {
        Logger::debug("Invalid pulse duration: expected %d us, got %d us",
                      _expected[i].pulseUs, _history[i].pulseUs);
        return false;
      }
    }
    Logger::debug("Preamble detected");
    _history.clear();
    return true;
  }

  /// Returns the expected preamble edges for this protocol.
  virtual int getEdges(
      pulsewire::Vector<pulsewire::OutputEdge>& output) const = 0;

  virtual size_t preambleLength() const = 0;

  /// Utility function to check if a value is within a specified tolerance range
  /// of a target
  bool inRange(uint32_t value, uint32_t target, uint32_t tolerance) const {
    return (value >= target - tolerance) && (value <= target + tolerance);
  }

 protected:
  // Default history buffer for edge detection
  pulsewire::Vector<pulsewire::OutputEdge> _history;
  pulsewire::Vector<pulsewire::OutputEdge> _expected;
};

/**
 * @brief NoPreamble: For protocols that do not require a preamble.
 * Always returns true for detect, does nothing for getEdges, and preambleLength
 * is 0.
 */
class NoPreamble : public Preamble {
 public:
  bool begin(uint32_t bitFrequencyHz) override { return true; }
  bool detect(const pulsewire::OutputEdge& edge) override { return true; }
  int getEdges(
      pulsewire::Vector<pulsewire::OutputEdge>& output) const override {
    return 0;
  }
  size_t preambleLength() const override { return 0; }
};
/**
 * @brief CustomPreambleUs: Allows users to define their own preamble by setting
 * expected edges. Useful for custom protocols or testing.
 *
 * @note The duration is specified in microseconds!
 *
 */
class CustomPreambleUs : public Preamble {
 public:
  CustomPreambleUs() = default;
  CustomPreambleUs(const pulsewire::Vector<pulsewire::OutputEdge>& edges) {
    _expected = edges;
  }
  // Uses default detect()
  void setEdges(const pulsewire::Vector<pulsewire::OutputEdge>& edges) {
    _expected = edges;
  }
  void addEdge(const pulsewire::OutputEdge& edge) { _expected.push_back(edge); }
  void addEdge(bool level, uint32_t pulseUs) {
    OutputEdge edge{level, pulseUs};
    _expected.push_back(edge);
  }
  void addEdge(bool level, uint16_t pulseCount, uint32_t frequencyHz) {
    uint32_t pulseUs = (1000000 / frequencyHz) * pulseCount;
    OutputEdge edge{level, pulseUs};
    _expected.push_back(edge);
  }
  void clear() { _expected.clear(); }
  int getEdges(Vector<OutputEdge>& output) const override {
    for (const auto& edge : _expected) {
      output.push_back(edge);
    }
    return _expected.size();
  }
  size_t preambleLength() const override { return _expected.size(); }
};

/**
 * @brief CustomPreamble: Allows users to define their own preamble by setting
 * expected edges. Useful for custom protocols or testing.
 *
 * @note The duration is specified in bit counts and will be converted to
 * microseconds
 *
 */
class CustomPreamble : public CustomPreambleUs {
 public:
  CustomPreamble() = default;
  CustomPreamble(const pulsewire::Vector<pulsewire::OutputEdge>& edges) {
    _expected = edges;
  }
  virtual bool begin(uint32_t bitFrequencyHz) {
    bool rc = Preamble::begin(bitFrequencyHz);
    uint32_t us = 1000000 / bitFrequencyHz;
    for (auto& edge : _expected) {
      edge.pulseUs = edge.pulseUs * us;
    }

    return rc;
  }
};

/**
 * @brief Custom Manchester preamble detector: run-in of alternating edges plus
 * unique start pulse.
 *
 * This preamble consists of several cycles of alternating HIGH/LOW pulses
 * (run-in), followed by a unique longer start pulse. This pattern is highly
 * reliable for Manchester encoding, as it provides both clock synchronization
 * and a unique start marker that cannot occur in normal Manchester data. The
 * detector is efficient: it only needs to check the last N edges for the
 * pattern, minimizing processing time.
 *
 * Example (bitPeriod = 500us, runInCycles = 4):
 *   [HIGH 500us, LOW 500us, HIGH 500us, LOW 500us, HIGH 500us, LOW 500us, HIGH
 * 500us, LOW 500us, HIGH 1000us]
 *
 * - runInCycles: Number of HIGH/LOW cycles for clock synchronization (default:
 * 4)
 * - bitPeriod: Duration of each bit (default: 500us)
 * - startPulse: Unique start pulse (default: HIGH, 2x bitPeriod)
 */
class ManchesterPreamble : public CustomPreambleUs {
 public:
  ManchesterPreamble(uint8_t runInCycles = 2) : _runInCycles(runInCycles) {}

  bool begin(uint32_t bitFrequencyHz) override {
    constexpr uint8_t MAX_RUN_IN_CYCLES = 32;
    if (_runInCycles == 0) {
      _runInCycles = 2;
    } else if (_runInCycles > MAX_RUN_IN_CYCLES) {
      Logger::error("ManchesterPreamble: runInCycles=%d too large, clamping to %d",
                    _runInCycles, MAX_RUN_IN_CYCLES);
      _runInCycles = MAX_RUN_IN_CYCLES;
    }

    // double the frequency for Manchester since each bit is two edges
    _history.clear();
    size_t runInEdges = static_cast<size_t>(_runInCycles) * 2;
    _history.reserve(runInEdges + 1);  // run-in edges + start pulse
    _expected.reserve(runInEdges + 1);
    _freqHz = bitFrequencyHz * 2;
    _bitPeriod = 1000000 / _freqHz;

    // Generate the expected edges for the Manchester preamble
    _expected.clear();
    bool level = true;
    // Run-in: alternating edges for clock recovery
    for (size_t i = 0; i < runInEdges; ++i) {
      addEdge(level, _bitPeriod);
      level = !level;
    }

    return true;
  }

 private:
  uint8_t _runInCycles = 0;
  uint32_t _freqHz = 0;
  uint32_t _bitPeriod = 0;
};

/**
 * @brief A Preamble implementation for NRZ protocols: alternating edges at full
 * bit period, ending with a HIGH level to match NRZ idle state. This provides a
 * clear synchronization pattern while ensuring the line is in the correct idle
 * state after the preamble, which is crucial for accurate decoding of NRZ
 * signals.
 */

class NRZPreamble : public CustomPreambleUs {
 public:
  NRZPreamble() = default;

  bool begin(uint32_t bitFrequencyHz) override {
    _history.clear();
    _history.reserve(4);
    _expected.clear();
    uint32_t bp = 1000000 / bitFrequencyHz;
    addEdge(true, bp);  // LOW
    addEdge(false, bp);   // HIGH
    addEdge(true, bp);  // LOW
    addEdge(false, bp);   // HIGH — ends HIGH, matching NRZ idle
    return true;
  }
};

}  // namespace pulsewire
