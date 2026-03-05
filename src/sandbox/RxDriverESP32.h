#pragma once
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stddef.h>
#include <stdint.h>

#include "TransceiverConfig.h"
#include "pulse/RxDriver.h"
#include "pulse/TxDriver.h"
#include "pulse/codecs/Codec.h"
#include "pulse/tools/Logger.h"
#include "pulse/tools/RingBuffer.h"
#include "pulse/tools/Vector.h"

namespace pulsewire {

/**
 * @brief High-performance ESP32 IR RX driver using the RMT peripheral and
 * Manchester decoding.
 *
 * - Uses the ESP32 RMT peripheral for efficient IR edge capture and decoding.
 * - Supports concurrent IR receive and transmit by using separate RMT channels
 * for RX and TX.
 * - RX and TX channels are independent; no conflicts as long as each driver
 * manages its own channel.
 * - Frame size and buffer size are configurable.
 * - Asynchronous reception is handled by a FreeRTOS task.
 * - Bit frequency (Hz) should match the transmitter for reliable decoding.
 *   - Typical values: 300–2400 Hz for consumer IR remotes, 1000–2000 Hz for
 * Arduino/ESP32 projects.
 *   - ESP32 RMT can handle higher frequencies, but receiver hardware and
 * software must be tuned accordingly.
 * - For Manchester encoding, bit frequency = baud rate (bits per second).
 *
 * @note Supports sending and receiving on the same microcontroller without
 * conflicts.
 */
class RxDriverESP32 : public RxDriver {
 public:
  /**
   * @param codec IR codec
   * @param pin RX pin
   * @param freqHz Bit frequency
   * @param useChecksum If true, validate checksum (default: false)
   * @param timeoutUs Timeout in microseconds to flush frame if no symbol
   * received (default: 5000)
   */
  RxDriverESP32(ManchesterCodec& codec, uint8_t pin,
                uint32_t freqHz = DEFAULT_BIT_FREQ_HZ, bool useChecksum = false,
                uint32_t timeoutUs = 5000)
      : _codec(codec),
        _rxPin(pin),
        _freqHz(freqHz),
        _useChecksum(useChecksum),
        _timeoutUs(timeoutUs) {
    _lastSymbolTime = 0;
  }

  void setFrameSize(uint16_t size) override { _frameSize = size; }

  void setRxBufferSize(size_t size) override { _rxBufferSize = size; }

  bool begin(uint16_t bitFrequencyHz) override {
    if (!_codec.begin(bitFrequencyHz)) {
      return false;
    }
    manchesterBits.reserve(_codec.getEdgeCount());
    // Enable DMA for large frames
    uint8_t dma = (_frameSize > 64) ? 1 : 0;
    rmt_rx_channel_config_t rx_config = {.gpio_num = (gpio_num_t)_rxPin,
                                         .clk_src = RMT_CLK_SRC_DEFAULT,
                                         .resolution_hz = 1000000,
                                         .mem_block_symbols = 64,
                                         .flags = {
                                             .invert_in = 0,
                                             .with_dma = dma,
                                         }};

    // Setup channel
    rmt_channel_handle_t rx_channel = nullptr;
    esp_err_t rmt_result = rmt_new_rx_channel(&rx_config, &rx_channel);
    if (rmt_result != ESP_OK || rx_channel == nullptr) {
      Logger::error("RMT RX channel initialization failed: %d", rmt_result);
      return false;
    }
    _rxChannel = rx_channel;

    // Setup FreeRTOS queue for received frames
    int count = _rxBufferSize / _frameSize;
    if (_rxBufferSize % _frameSize != 0) count++;

    // Create Queue
    _frameQueue = xQueueCreate(count, _frameSize);  // n frames max in queue
    if (_frameQueue == nullptr) {
      Logger::error("Failed to create frame queue");
      return false;
    }
    rxOverflow.resize(_rxBufferSize);
    _stopTask = false;

    // Create task
    BaseType_t taskResult = xTaskCreatePinnedToCore(rxTask, "rmt_rx", 4096,
                                                    this, 1, &_taskHandle, 1);
    if (taskResult != pdPASS || _taskHandle == nullptr) {
      Logger::error("Failed to create RX task: %d", taskResult);
      return false;
    }
    return true;
  }

  void end() override {
    _stopTask = true;
    if (_taskHandle) {
      // Wait for task to exit
      while (eTaskGetState(_taskHandle) != eDeleted) {
        delay(1);
      }
      _taskHandle = nullptr;
    }
    if (_rxChannel) {
      rmt_del_channel(_rxChannel);
      _rxChannel = nullptr;
    }
  }

  size_t readBytes(uint8_t* buffer, size_t length) override {
    size_t totalRead = 0;
    // First, serve from rxOverflow if any
    while (length > 0 && rxOverflow.available() > 0) {
      int b = rxOverflow.read();
      if (b < 0) break;
      *buffer++ = (uint8_t)b;
      --length;
      ++totalRead;
    }
    // Then, read full frames from the queue as needed
    uint8_t frame[_frameSize]{};
    while (length > 0 && _frameQueue &&
           uxQueueMessagesWaiting(_frameQueue) > 0) {
      if (xQueueReceive(_frameQueue, frame, 0) == pdTRUE) {
        size_t toCopy = (length < _frameSize) ? length : _frameSize;
        memcpy(buffer, frame, toCopy);
        totalRead += toCopy;
        buffer += toCopy;
        length -= toCopy;
        // If we didn't consume the whole frame, store the rest in rxOverflow
        if (toCopy < _frameSize) {
          rxOverflow.writeArray(frame + toCopy, _frameSize - toCopy);
        }
      }
    }
    return totalRead;
  }

  int available() override {
    if (!_frameQueue) return 0;
    return uxQueueMessagesWaiting(_frameQueue) * _frameSize;
  }

 protected:
  size_t _rxBufferSize = 256;
  uint8_t _rxPin;
  uint16_t _frameSize = DEFAULT_FRAME_SIZE;
  uint32_t _freqHz;
  rmt_channel_handle_t _rxChannel = nullptr;
  TaskHandle_t _taskHandle = nullptr;
  volatile bool _stopTask = false;
  uint16_t _preambleBits = 0;
  bool _inFrame = false;
  QueueHandle_t _frameQueue = nullptr;
  RingBuffer<uint8_t> rxOverflow;
  ManchesterCodec& _codec;
  Vector<uint8_t> manchesterBits;
  bool _useChecksum = false;
  uint32_t _timeoutUs = 5000;
  uint32_t _lastSymbolTime = 0;

  /**
   * @brief FreeRTOS task for receiving and decoding IR frames using ESP32 RMT.
   *
   * This method is protocol-agnostic: it processes each RMT symbol by summing
   * durations and inferring the logic level, then passes these to the codec's
   * decodeEdge method. As long as the codec implements decodeEdge for its
   * protocol, this works for Manchester, PulseWidth, PulseDistance, etc.
   *
   * @param arg Pointer to RxDriverESP32 instance.
   */
  static void rxTask(void* arg) {
    auto* self = static_cast<RxDriverESP32*>(arg);
    size_t symbolCount = self->_frameSize * self->_codec.getEdgeCount();
    int frameSize = self->_frameSize;
    Vector<rmt_symbol_word_t> symbols(symbolCount);
    uint8_t data;
    Vector<uint8_t> frame;
    frame.reserve(frameSize);

    self->_lastSymbolTime = micros();
    while (!self->_stopTask) {
      size_t rx_size = 0;
      rmt_receive_config_t rx_cfg = {};
      esp_err_t err =
          rmt_receive(self->_rxChannel, symbols.data(),
                      symbols.size() * sizeof(rmt_symbol_word_t), &rx_cfg);
      bool gotSymbol = false;
      if (err == ESP_OK && rx_size > 0) {
        size_t num_bits = rx_size / sizeof(rmt_symbol_word_t);
        // Use decodeEdge for each symbol
        uint32_t bitPeriodUs = 1000000UL / self->_freqHz;
        uint32_t minUs = bitPeriodUs / 2;
        uint32_t maxUs = bitPeriodUs * 2;
        size_t frameLen = 0;
        for (size_t i = 0; i < num_bits; ++i) {
          uint32_t duration = symbols[i].duration0 + symbols[i].duration1;
          bool isOne = (symbols[i].level0 == 1 && symbols[i].level1 == 0);
          bool isZero = (symbols[i].level0 == 0 && symbols[i].level1 == 1);
          if (isOne || isZero) {
            bool level = isOne ? 1 : 0;
            self->_lastSymbolTime = micros();
            gotSymbol = true;
            bool isOK = true;
            if (self->_codec.decodeEdge(duration, level, data)) {
              frame.push_back(data);

              if (frame.size() == frameSize) {
                if (self->_useChecksum) {
                  // Validate checksum
                  uint8_t sum = 0;
                  for (size_t j = 0; j < frameLen - 1; ++j) sum += frame[j];
                  isOK = (sum == frame[frameLen - 1]);
                }
                if (isOK) {
                  xQueueSend(self->_frameQueue, frame.data(), 0);
                }
                frame.clear();
                memset(frame.data(), 0, frame.capacity());
              }
            }
          }
        }
        // Timeout logic: if no symbol received for timeoutUs, flush buffer
        uint32_t now = micros();
        if (!gotSymbol && (now - self->_lastSymbolTime > self->_timeoutUs) && !frame.empty()) {
          // Signal end-of-frame to codec
          if (self->_codec.decodeEdge(self->_codec.getEndOfFrameDelayUs(), self->_codec.getIdleLevel(), data)) {
            frame.push_back(data);
            if (self->_useChecksum) {
              if (frame.size() >= 2) {
                uint8_t sum = 0;
                for (size_t j = 0; j < frame.size() - 1; ++j) sum += frame[j];
                if (sum == frame[frame.size() - 1]) {
                  xQueueSend(self->_frameQueue, frame.data(), 0);
                }
              }
            } else {
              xQueueSend(self->_frameQueue, frame.data(), 0);
            }
            frame.clear();
            memset(frame.data(), 0, frame.capacity());
          }
          self->_lastSymbolTime = now;
        }
      }
      delay(1);
    }
    vTaskDelete(nullptr);
  }

  // processSymbols is no longer needed; decoding is handled by decodeEdge
};

}  // namespace pulsewire
