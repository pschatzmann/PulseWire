#pragma once
#include <Arduino.h>

#include "pulse/Codec.h"
#include "pulse/RxDriver.h"
#include "pulse/Vector.h"

namespace pulsewire {

/**
 * @brief High-performance RP2040 IR RX driver using PIO for edge capture.
 *
 * Uses a PIO state machine to capture IR signal edges and their timing.
 * Buffers edge durations and levels for protocol-agnostic decoding.
 * Compatible with modular Codec/Preamble architecture.
 * @note Supports sending and receiving on the same microcontroller without
 * conflicts.
 */
class RxDriverRP2040 : public RxDriver {
 public:
  /**
   * @param codec IR codec
   * @param pin RX pin
   * @param freqHz Bit frequency
   * @param useChecksum If true, validate checksum (default: false)
   * @param timeoutUs Timeout in microseconds to flush frame if no edge received
   * (default: 1000000)
   */
  RxDriverRP2040(Codec& codec, uint8_t pin, uint32_t freqHz = 1000,
                 bool useChecksum = false, uint32_t timeoutUs = 1000000)
      : _codec(codec),
        _rxPin(pin),
        _freqHz(freqHz),
        _useChecksum(useChecksum),
        _timeoutUs(timeoutUs) {
    _lastEdgeTime = micros();
  }

  void setFrameSize(uint16_t size) override { _frameSize = size; }

  bool begin(uint8_t frameSize) override {
    setFrameSize(frameSize);
    return begin();
  }

  bool begin() override {
    _codec.begin();
    // DMA setup
    dmaChannel = dma_claim_unused_channel(true);
    if (dmaChannel < 0) return false;
    dma_channel_config cfg = dma_channel_get_default_config(dmaChannel);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    bool dma_config_ok = dma_channel_configure(
        dmaChannel, &cfg, dmaBuf, &pio0->rxfifo[0], DMA_BUF_SIZE, false);
    if (!dma_config_ok) return false;
    dma_channel_start(dmaChannel);
    // Pin initialization
    pinMode(_rxPin, INPUT);
    // PIO setup
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    if (sm == (uint)-1) return false;
    uint offset = pio_add_program(pio, (const pio_program_t*)&ir_rx_program);
    if (offset == (uint)-1) return false;
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, _rxPin);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_set_consecutive_pindirs(pio, sm, _rxPin, 1, false);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
    return true;
  }

  void pollPIO() {
    // Poll DMA buffer for new edge timings
    while (dmaBufHead < DMA_BUF_SIZE) {
      uint32_t data = dmaBuf[dmaBufHead++];
      uint32_t duration = data & 0xFFFFFF;
      bool level = (data >> 24) & 0x01;
      _edgeDurations.push_back(duration);
      _edgeLevels.push_back(level);
      _lastEdgeTime = micros();
    }
    // Reset head if buffer is full
    if (dmaBufHead >= DMA_BUF_SIZE) dmaBufHead = 0;
  }

  void checkTimeout() {
    uint32_t now = micros();
    if (!_edgeDurations.empty() && (now - _lastEdgeTime > _timeoutUs)) {
      // Try to decode with a zero-duration edge to flush the buffer
      uint8_t buffer[32] = {0};
      size_t frameLen = 0;
      if (_codec.decodeEdge(0, 0, 0, 0, buffer, _frameSize, frameLen)) {
        bool valid = true;
        if (_useChecksum) {
          if (frameLen < 2)
            valid = false;
          else {
            uint8_t sum = 0;
            for (size_t j = 0; j < frameLen - 1; ++j) sum += buffer[j];
            valid = (sum == buffer[frameLen - 1]);
          }
        }
        if (valid) {
          for (size_t i = 0; i < frameLen; ++i)
            _frameBuffer.push_back(buffer[i]);
        }
      }
      _edgeDurations.clear();
      _edgeLevels.clear();
      _lastEdgeTime = now;
    }
  }

  size_t readBytes(uint8_t* buffer, size_t length) override {
    checkTimeout();
    size_t totalRead = 0;
    // Poll edge buffer and decode frames using codec
    while (length > 0 && available() >= _frameSize) {
      if (decodeFrame(buffer, _frameSize)) {
        buffer += _frameSize;
        length -= _frameSize;
        totalRead += _frameSize;
      } else {
        break;
      }
    }
    return totalRead;
  }

  int available() override {
    checkTimeout();
    // Return number of bytes available in decoded frame buffer
    return _frameBuffer.size();
  }

 protected:
  Codec& _codec;
  uint8_t _rxPin;
  uint32_t _freqHz;
  uint16_t _frameSize = 20;
  Vector<uint8_t> _frameBuffer;
  Vector<uint32_t> _edgeDurations;
  Vector<bool> _edgeLevels;
  // DMA support
  static constexpr size_t DMA_BUF_SIZE = 128;
  uint32_t dmaBuf[DMA_BUF_SIZE];
  volatile size_t dmaBufHead = 0;
  int dmaChannel = -1;
  bool _useChecksum = false;
  uint32_t _timeoutUs = 1000000;
  uint32_t _lastEdgeTime = 0;
  // Simple PIO program to capture edges and timings
  static const uint16_t ir_rx_program[4] = {
      0x2020,  // wait for pin high
      0xa042,  // in pins, 1
      0x2021,  // wait for pin low
      0xa042,  // in pins, 1
  };

  // Example decodeFrame implementation
  bool decodeFrame(uint8_t* buffer, size_t frameSize) {
    // Use edgeDurations and edgeLevels to call codec.decodeEdge
    size_t frameLen = 0;
    for (size_t i = 0; i < _edgeDurations.size(); ++i) {
      if (_codec.decodeEdge(_edgeDurations[i], _edgeLevels[i], 0, 0, buffer,
                            frameSize, frameLen)) {
        bool valid = true;
        if (_useChecksum) {
          if (frameLen < 2)
            valid = false;
          else {
            uint8_t sum = 0;
            for (size_t j = 0; j < frameLen - 1; ++j) sum += buffer[j];
            valid = (sum == buffer[frameLen - 1]);
          }
        }
        if (valid) {
          // Remove used edges
          _edgeDurations.erase(_edgeDurations.begin(),
                               _edgeDurations.begin() + i + 1);
          _edgeLevels.erase(_edgeLevels.begin(), _edgeLevels.begin() + i + 1);
          return true;
        }
      }
    }
    return false;
  }
};

}  // namespace pulsewire
