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

bool rmt_rx_done_callback(rmt_channel_handle_t channel,
                          const rmt_rx_done_event_data_t* edata,
                          void* user_data) {
  BaseType_t high_task_wakeup = pdFALSE;
  QueueHandle_t receive_queue = (QueueHandle_t)user_data;
  // send the received RMT symbols to the parser task
  xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
  // return whether any task is woken up
  return high_task_wakeup == pdTRUE;
}

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
  RxDriverESP32(ManchesterCodec& codec, uint8_t pin, bool useChecksum = false)
      : _codec(codec), _rxPin(pin), _useChecksum(useChecksum) {}

  void setFrameSize(uint16_t size) override { _frameSize = size; }

  void setRxBufferSize(size_t size) override { _rxBufferSize = size; }

  bool begin(uint32_t bitFrequencyHz) override {
    Logger::info("begin: RxDriverESP32 with bitFrequencyHz=%d, pin=%d",
                 bitFrequencyHz, _rxPin);
    if (isActive && _freqHz == bitFrequencyHz) {
      return true;
    }
    if (isActive) {
      Logger::info("Reinitializing RxDriverESP32 with new frequency: %d Hz",
                   bitFrequencyHz);
      end();
    }
    _freqHz = bitFrequencyHz;
    if (!_codec.begin(bitFrequencyHz)) {
      return false;
    }
    size_t symbols = _frameSize * 8;
    int n64 = (symbols / 64) + 1;
    symbols = n64 * 64;

    if (n64 > 7){
      Logger::error("Frame size too large for RMT buffer: %d symbols (max 448)",
                    symbols);
      return false;
    }

    uint8_t dma = (_frameSize > 64) ? 1 : 0;
    rmt_rx_channel_config_t rx_config = {.gpio_num = (gpio_num_t)_rxPin,
                                         .clk_src = RMT_CLK_SRC_DEFAULT,
                                         .resolution_hz = 1000000,
                                         .mem_block_symbols = symbols,
                                         .flags = {
                                             .invert_in = 0,
                                             .with_dma = dma,
                                         }};

    // Setup channel, try with DMA first if requested
    rmt_channel_handle_t rx_channel = nullptr;
    esp_err_t rmt_result = rmt_new_rx_channel(&rx_config, &rx_channel);
    if ((rmt_result != ESP_OK || rx_channel == nullptr) && dma) {
      Logger::warning(
          "RMT RX channel with DMA failed (%d), retrying without DMA",
          rmt_result);
      rx_config.flags.with_dma = 0;
      rmt_result = rmt_new_rx_channel(&rx_config, &rx_channel);
    }
    if (rmt_result != ESP_OK || rx_channel == nullptr) {
      Logger::error("RMT RX channel initialization failed: %d", rmt_result);
      return false;
    }
    _rxChannel = rx_channel;

    esp_err_t enable_result = rmt_enable(_rxChannel);
    if (enable_result != ESP_OK) {
      Logger::error("Failed to enable RX channel: %d", enable_result);
      return false;
    }

    receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    esp_err_t cb_result =
        rmt_rx_register_event_callbacks(_rxChannel, &cbs, receive_queue);
    if (cb_result != ESP_OK) {
      Logger::error("Failed to register RX event callbacks: %d", cb_result);
      return false;
    }

    _rxBuffer.resize(_rxBufferSize);
    _symbols.resize(symbols);  // Each symbol has 2 edges, +1 for safety margin
    _edgeBuffer.resize(
        _frameSize *
        _codec.getEdgeCount());  // Ensure edge buffer can hold all edges
    Logger::info("_frameSize=%d", _frameSize);
    Logger::info("_rxBufferSize=%d", _rxBufferSize);
    Logger::info("_symbols size=%d", _symbols.size());
    Logger::info("_edgeBuffer size=%d", _edgeBuffer.size());
    isActive = true;
    return true;
  }

  void end() override {
    if (_rxChannel) {
      rmt_disable(_rxChannel);
      rmt_del_channel(_rxChannel);
      _rxChannel = nullptr;
    }
    isActive = false;
  }

  size_t readBytes(uint8_t* buffer, size_t length) override {
    if (_rxBuffer.available() == 0) {
      // No data available, attempt to read a new frame
      readFrame();
    }
    return _rxBuffer.readArray(buffer, length);
  }

  int available() override {
    if (_rxBuffer.available() == 0) {
      // No data available, attempt to read a new frame
      readFrame();
    }
    return _rxBuffer.available();
  }

 protected:
  size_t _rxBufferSize = 1024;
  size_t _frameSize = 0;
  uint8_t _rxPin;
  uint32_t _freqHz;
  rmt_channel_handle_t _rxChannel = nullptr;
  RingBuffer<uint8_t> _rxBuffer;
  RingBuffer<OutputEdge> _edgeBuffer;
  Vector<rmt_symbol_word_t> _symbols;
  Codec& _codec;
  bool _useChecksum = false;
  bool isActive = false;
  QueueHandle_t receive_queue;

  size_t readFrame() {
    size_t totalRead = 0;
    if (_rxChannel == nullptr) {
      Logger::error("readFrame() called but RX channel is not initialized");
      return 0;
    }

    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = 10,
        .signal_range_max_ns =
            static_cast<uint32_t>(_codec.getEndOfFrameDelayUs() * 1000),
    };

    rmt_symbol_word_t* symbols_data = _symbols.data();
    size_t symbols_size = (_symbols.size()) * sizeof(rmt_symbol_word_t);

    Logger::debug("cm_receive: waiting for frame (max %d symbols)",
                  _symbols.size());
    esp_err_t err =
        rmt_receive(_rxChannel, symbols_data, symbols_size, &rx_cfg);
    if (err != ESP_OK) {
      Logger::error("RMT receive failed: %d", err);
      return 0;
    }

    // wait for the RX-done signal
    rmt_rx_done_event_data_t rx_data{};
    esp_err_t esp_err = xQueueReceive(receive_queue, &rx_data,
                                      2 * _codec.getEndOfFrameDelayUs() / 1000);
    if (esp_err != pdTRUE) {
      Logger::error("Failed to receive RMT data from queue: %d", esp_err);
      return 0;
    }

    // Process symbols into edges and feed into edge buffer
    toEdges(rx_data.received_symbols, rx_data.num_symbols, _edgeBuffer);

    Logger::debug("Received %d symbols, converted to %d edges",
                  rx_data.num_symbols, _edgeBuffer.available());
    // process edges into codec and fill RX buffer
    _codec.reset();
    while (!_edgeBuffer.isEmpty()) {
      OutputEdge edge;
      _edgeBuffer.read(edge);
      Logger::debug("RX edge: level=%d duration=%d us", edge.level,
                    edge.pulseUs);
      uint8_t data = 0;
      if (_codec.decodeEdge(edge.pulseUs, edge.level, data)) {
        if (_rxBuffer.write(data) == 0) {
          Logger::error("RX buffer overflow: size=%d, available=%d",
                        _rxBuffer.size(), _rxBuffer.available());
        }
      }
    }
    return _rxBuffer.available();
  }

  /// Convert RMT symbols to edges and feed into codec
  void toEdges(rmt_symbol_word_t* symbols, size_t num_symbols,
               RingBuffer<OutputEdge>& edges) {
    Logger::debug("toEdges: converting %d symbols to edges", num_symbols);
    bool is_end = false;
    int processed = 0;
    for (size_t i = 0; i < num_symbols; ++i) {
      bool level = symbols[i].level0;
      uint32_t duration = symbols[i].duration0;
      edges.write(OutputEdge(level, duration));

      level = symbols[i].level1;
      duration = symbols[i].duration1;
      edges.write(OutputEdge(level, duration));
    }
  }
};

}  // namespace pulsewire
