#pragma once
#include <Arduino.h>

#include "Codec.h"
#include "ManchesterCodec.h"
#include "RingBuffer.h"
#include "RxDriver.h"
#include "TransceiverConfig.h"
#include "pulse/Logger.h"
#include "pulse/Vector.h"

namespace pulsewire {

/**
 * @brief Interrupt-capable IR RX driver interface for platforms without
 * attachInterruptArg.
 *
 * This class extends RxDriver with a pure virtual method handleInterrupt(),
 * which must be implemented by derived classes to handle the interrupt logic.
 * It is used in conjunction with ISRManager to manage multiple ISR instances on
 * platforms that do not support attachInterruptArg (e.g., AVR).
 */
class RxDriverInt : public RxDriver {
 public:
  virtual void handleInterrupt() = 0;
};

#if !defined(HAS_INTERRUPT_ARG)

/**
 * @brief Manager for handling multiple ISR instances on platforms without
 * attachInterruptArg.
 */
class ISRManager {
 public:
  static bool attach(uint8_t pin, RxDriverInt* instance) {
    initISR();
    for (int i = 0; i < 10; i++) {
      if (!_isrData[i].active || _isrData[i].pin == pin) {
        _isrData[i].instance = instance;
        _isrData[i].active = true;
        _isrData[i].pin = pin;
        attachInterrupt(digitalPinToInterrupt(pin), _isrData[i].isr, CHANGE);
        return true;
      }
    }
    return false;
  }

  static bool isAttached(uint8_t pin) {
    for (int i = 0; i < 10; i++) {
      if (_isrData[i].active && _isrData[i].pin == pin) {
        return true;
      }
    }
    return false;
  }

  static void detach(uint8_t pin) {
    for (int i = 0; i < 10; i++) {
      if (_isrData[i].active && _isrData[i].pin == pin) {
        detachInterrupt(digitalPinToInterrupt(pin));
        _isrData[i].instance = nullptr;
        _isrData[i].active = false;
        _isrData[i].pin = -1;
        break;
      }
    }
  }

 private:
  struct InstanceData {
    RxDriverInt* instance;
    void (*isr)(void);
    int pin;
    bool active;
  };

  static InstanceData _isrData[10];

  static void isr0() { dispatch(0); }
  static void isr1() { dispatch(1); }
  static void isr2() { dispatch(2); }
  static void isr3() { dispatch(3); }
  static void isr4() { dispatch(4); }
  static void isr5() { dispatch(5); }
  static void isr6() { dispatch(6); }
  static void isr7() { dispatch(7); }
  static void isr8() { dispatch(8); }
  static void isr9() { dispatch(9); }

  static void initISR() {
    static bool initialized = false;
    if (!initialized) {
      void (*isrTable[10])() = {isr0, isr1, isr2, isr3, isr4,
                                isr5, isr6, isr7, isr8, isr9};
      for (int i = 0; i < 10; ++i) {
        _isrData[i].instance = nullptr;
        _isrData[i].isr = isrTable[i];
        _isrData[i].pin = -1;
        _isrData[i].active = false;
      }
      initialized = true;
    }
  }

  static void dispatch(int index) {
    if (index >= 0 && index < 10 && _isrData[index].active) {
      _isrData[index].instance->handleInterrupt();
    }
  }
};

#endif

/**
 * @brief Interrupt-driven Arduino RX driver for pulse-based protocols.
 *
 * RxDriverArduino receives and decodes pulse-based signals (such as IR or
 * single-wire protocols) using hardware interrupts for precise edge timing. It
 * supports any Codec (Manchester, NRZ, etc.) and is compatible with both
 * platforms that support attachInterruptArg (e.g., ESP32, RP2040) and those
 * that do not (e.g., AVR, via ISRManager).
 *
 * Features:
 *   - Interrupt-driven edge capture for reliable timing
 *   - Works with any Codec (Manchester, NRZ, PWM, etc.)
 *   - Configurable RX pin, bit frequency, frame size, and optional checksum
 *   - Automatic timing window calculation for robust decoding
 *   - Buffering of received bytes and edges
 *   - Timeout and frame boundary detection
 *   - Supports up to 10 concurrent instances on platforms without
 * attachInterruptArg
 *
 * Limitations:
 *   - On platforms without attachInterruptArg, only 10 instances can be active
 * (one per RX pin)
 *   - Maximum reliable bit rate depends on hardware and signal quality
 * (typically up to 2–3 kbit/s)
 */
class RxDriverArduino : public RxDriverInt {
 public:
  /**
   * @param codec IR codec
   * @param pin RX pin
   * @param freqHz Bit frequency
   * @param useChecksum If true, validate checksum (default: false)
   */

  RxDriverArduino(Codec& codec, uint8_t pin,
                  uint32_t freqHz = DEFAULT_BIT_FREQ_HZ,
                  bool useChecksum = false, uint32_t timeoutUs = 0)
      : _pin(pin),
        _codec(codec),
        _freqHz(freqHz),
        _useChecksum(useChecksum),
        _timeoutUs(timeoutUs) {
    reset();
  }

  void init(Codec& codec, uint8_t pin, uint32_t freqHz = DEFAULT_BIT_FREQ_HZ,
            bool useChecksum = false, uint32_t timeoutUs = 0) {
    TRACE();
    _pin = pin;
    _codec = codec;
    _freqHz = freqHz;
    _useChecksum = useChecksum;
    _timeoutUs = timeoutUs;
    reset();
  }

  //  destructor
  ~RxDriverArduino() { end(); }

  void setFrameSize(uint16_t size) override {
    TRACE();
    _codec.setFrameSize(size);
    _frameSize = size;
    _rxBuffer.resize(size * 2);
    _edgeBuffer.resize(size * _codec.getEdgeCount() *
                       2);  // enough for edges of a full frame
  }

  void setRxBufferSize(size_t size) { _rxBuffer.resize(size); }

  bool begin(uint16_t bitFrequencyHz = DEFAULT_BIT_FREQ_HZ) override {
    TRACE();
    _freqHz = bitFrequencyHz;

    // Calculate bit timing window from freqHz
    _bitPeriodUs = 1000000UL / _freqHz;
    _minUs = _bitPeriodUs / 2;
    _maxUs = _bitPeriodUs * 2;
    Logger::info(
        "Bit frequency: %d Hz, Bit period: %d us, Timing window: [%d, %d] us",
        _freqHz, _bitPeriodUs, _minUs, _maxUs);

    if (_timeoutUs == 0) {
      _timeoutUs = 0.95 *  _codec.getEndOfFrameDelayUs();  

    }

    // Ensure byte buffer is sized for frame
    setFrameSize(_frameSize);

    // Initialize codec once
    bool rc = _codec.begin(bitFrequencyHz);

    if (!_is_active) {
      // Reset state BEFORE attaching interrupt
      reset();
      // Read initial pin level before first interrupt
      pinMode(_pin, INPUT);
      _lastLevel = digitalRead(_pin);
      _lastEdge = micros();

#ifdef HAS_INTERRUPT_ARG
      attachInterruptArg(_pin, interruptHandler, this, CHANGE);
#else
      if (!ISRManager::attach(_pin, this)) {
        return false;
      }
#endif
      _is_active = true;
    }

    Logger::info("RX driver started on pin %d with codec %s with frame size %d",
                 _pin, _codec.name(), _frameSize);
    return rc;
  }

  size_t readBytes(uint8_t* buffer, size_t length) override {
    processEdges();
    checkTimeout();
    if (_rxBuffer.size() == 0) {
      return 0;
    }
    size_t count = 0;
    count = _rxBuffer.readArray((uint8_t*)buffer, length);
    return count;
  }

  int available() override {
    processEdges();
    checkTimeout();
    int result = _rxBuffer.available();
    Logger::debug("available(): %d", result);
    return result;
  }

  void end() {
    TRACE();
    if (_is_active) {
#ifdef HAS_INTERRUPT_ARG
      detachInterrupt(_pin);
#else
      ISRManager::detach(_pin);
#endif
      _is_active = false;
    }
  }

 protected:
  volatile uint32_t _lastEdge;
  volatile bool _lastLevel;
  uint8_t _pin;
  uint32_t _freqHz;
  uint16_t _frameSize = DEFAULT_FRAME_SIZE;
  Codec& _codec;
  RingBuffer<uint8_t> _rxBuffer;
  RingBuffer<OutputEdge> _edgeBuffer;

  uint32_t _bitPeriodUs = 0;
  uint32_t _minUs = 0;
  uint32_t _maxUs = 0;
  bool _useChecksum = false;
  uint32_t _timeoutUs = 0;
  volatile bool _is_active = false;
  volatile bool _is_open = false;

#ifdef HAS_INTERRUPT_ARG
  static IRAM_ATTR void interruptHandler(void* arg) {
    static_cast<RxDriverArduino*>(arg)->handleInterrupt();
  }
#endif

  void IRAM_ATTR handleInterrupt() override {
    bool newLevel = digitalRead(_pin);
    uint32_t now = micros();
    uint32_t duration = now - _lastEdge;
    _lastEdge = now;

    // Pass the duration and the previous level (_lastLevel)
    OutputEdge edge{_lastLevel, duration};
    _edgeBuffer.write(edge);
    _lastLevel = newLevel;
  }

  // Called from loop() context — safe for heap, virtual calls, etc.
  void processEdges() {
    Logger::debug("processEdges");
    bool ok = true;
    OutputEdge edge;
    while (!_rxBuffer.isFull() && ok) {
      noInterrupts();
      ok = _edgeBuffer.read(edge);
      interrupts();
      uint8_t byte_data = 0;
      if (ok && _codec.decodeEdge(edge.pulseUs, edge.level, byte_data)) {
        _rxBuffer.write(byte_data);
        _is_open = true;
      }
    }
  }

  void checkTimeout() {
    Logger::debug("checkTimeout");
    uint32_t now = micros();
    uint32_t duration = now - _lastEdge;
    uint8_t byte_data = 0;

    // Process pending final edge (e.g., last stop bit)
    if (_is_open) {
      // Protect codec state from ISR preemption
      bool has_data = _codec.decodeEdge(_bitPeriodUs, _lastLevel, byte_data);
      if (has_data) {
        _rxBuffer.write(byte_data);
      }
      _is_open = false;
    }

    // Process timeout — reset state atomically
    if (duration > _timeoutUs) {
      // noInterrupts();
      _codec.reset();
      // interrupts();
      //  Don't touch _lastEdge/_lastLevel here — ISR owns them
    }
  }

  void reset() {
    _lastEdge = micros();
    _lastLevel = 0;
    _is_open = false;
  }
};

}  // namespace pulsewire