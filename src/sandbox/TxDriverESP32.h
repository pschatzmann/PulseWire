
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
#include "pulse/TxDriverCommon.h"
#include "pulse/TxProtocol.h"
#include "pulse/codecs/Codec.h"
#include "pulse/tools/RingBuffer.h"
#include "pulse/tools/Vector.h"

namespace pulsewire {

/**
 * @brief ESP32-specific TxProtocol implementation using RMT for precise timing
 * and DMA support.
 */
class TxProtocolESP32 : public TxProtocol {
 public:
  TxProtocolESP32() = default;

  void setFrameSize(uint16_t size) {
    _frameSize = size;
    _itemsBuffer.resize(
        size * 2);  // Each byte can produce up to 2 edges (for Manchester)
  }

  void setCarrierHz(uint16_t carrierHz) { _carrierHz = carrierHz; }

  bool begin(uint16_t bitFrequencyHz, Codec* p_codec, uint8_t pin) {
    this->_codec = p_codec;
    this->_txPin = pin;
    if (!_codec->begin(bitFrequencyHz)) {
      Logger::error("Codec initialization failed");
      return false;
    }
    // Enable DMA for large frames
    uint8_t dma = (_frameSize > 64) ? 1 : 0;
    rmt_tx_channel_config_t tx_config = {.gpio_num = (gpio_num_t)_txPin,
                                         .clk_src = RMT_CLK_SRC_DEFAULT,
                                         .resolution_hz = 1000000,
                                         .trans_queue_depth = 4,
                                         .flags = {
                                             .invert_out = 0,
                                             .with_dma = dma,
                                         }};

    rmt_channel_handle_t tx_channel = nullptr;
    esp_err_t tx_result = rmt_new_tx_channel(&tx_config, &tx_channel);
    if (tx_result != ESP_OK || tx_channel == nullptr) {
      Logger::error("RMT TX channel initialization failed");
      return false;
    }
    // Set carrier frequency and duty cycle for IR
    if (_carrierHz > 0) {
      rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = _carrierHz, // e.g. 38000 for 38kHz
        .duty_cycle = 33,           // 33% duty cycle
      };
      esp_err_t carrier_result = rmt_apply_carrier(tx_channel, &carrier_cfg);
      if (carrier_result != ESP_OK) {
        Logger::error("RMT carrier config failed");
        return false;
      }
    }
    // Encoder setup
    rmt_bytes_encoder_config_t encoder_config = {};
    rmt_encoder_handle_t encoder = nullptr;
    esp_err_t enc_result = rmt_new_bytes_encoder(&encoder_config, &encoder);
    if (enc_result != ESP_OK || encoder == nullptr) {
      Logger::error("RMT encoder initialization failed");
      return false;
    }
    _txEncoder = encoder;
    // Carrier config
    rmt_transmit_config_t transmit_config = {.loop_count = 0, .flags = {}};
    _txChannel = tx_channel;
    _txTransmitConfig = transmit_config;
    return true;
  }

  void sendPreamble() override {
    if (is_frame_closed) {
      sum = 0;
      output.clear();
      // Insert preamble edges at the start
      _codec->getPreamble().getEdges(output);
      is_frame_closed = false;
      sum = 0;
    }
  }

  void sendData(const uint8_t* data, uint8_t len) override {
    if (len == 0) return;
    // process bytes
    for (int byteIdx = 0; byteIdx < len; ++byteIdx) {
      uint8_t b = data[byteIdx];
      sum += b;
      _codec->encode(b, output);
    }

    // Convert OutputSpec to RMT symbols (step by 2)
    size_t symbolCount = output.size() / 2;
    if (_itemsBuffer.size() < symbolCount) _itemsBuffer.resize(symbolCount);
    for (size_t i = 0, j = 0; i + 1 < output.size(); i += 2, ++j) {
      _itemsBuffer[j].duration0 = output[i].pulseUs;
      _itemsBuffer[j].level0 = output[i].level ? 1 : 0;
      _itemsBuffer[j].duration1 = output[i + 1].pulseUs;
      _itemsBuffer[j].level1 = output[i + 1].level ? 1 : 0;
    }
    if (_txChannel && _txEncoder) {
      esp_err_t err = rmt_transmit(_txChannel, _txEncoder, _itemsBuffer.data(),
                                   symbolCount * sizeof(rmt_symbol_word_t),
                                   &_txTransmitConfig);
      if (err != ESP_OK) {
        Logger::error("RMT transmission failed: %d", err);
      }
    }
    output.clear();
    _itemsBuffer.clear();
  }

  void sendEnd(bool& useChecksum, bool isDelayAfterFrame) {
    // Optionally append checksum
    if (is_frame_closed) return;
    if (useChecksum) {
      _codec->encode(sum, output);
      sum = 0;
    }

    // add delay
    if (isDelayAfterFrame) {
      output.push_back(
          OutputEdge(_codec->getEndOfFrameDelayUs(), _codec->getIdleLevel()));
      output.push_back(OutputEdge(0, 0));
    }
    // Convert OutputSpec to RMT symbols
    size_t symbolCount = output.size() / 2;
    if (_itemsBuffer.size() < symbolCount) _itemsBuffer.resize(symbolCount);
    for (size_t i = 0, j = 0; i + 1 < output.size(); i += 2, ++j) {
      _itemsBuffer[j].duration0 = output[i].pulseUs;
      _itemsBuffer[j].level0 = output[i].level ? 1 : 0;
      _itemsBuffer[j].duration1 = output[i + 1].pulseUs;
      _itemsBuffer[j].level1 = output[i + 1].level ? 1 : 0;
    }
    if (_txChannel && _txEncoder) {
      esp_err_t err = rmt_transmit(_txChannel, _txEncoder, _itemsBuffer.data(),
                                   symbolCount * sizeof(rmt_symbol_word_t),
                                   &_txTransmitConfig);
      if (err != ESP_OK) {
        Logger::error("RMT transmission failed: %d", err);
      }
    }
    is_frame_closed = true;
  }

  bool isFrameClosed() const override { return is_frame_closed; }

 protected:
  Codec* _codec = nullptr;
  uint8_t _txPin;
  uint16_t _frameSize = 64;
  rmt_channel_handle_t _txChannel = nullptr;
  Vector<rmt_symbol_word_t> _itemsBuffer;
  rmt_encoder_handle_t _txEncoder = nullptr;
  rmt_transmit_config_t _txTransmitConfig = {};
  Vector<OutputEdge> output;
  uint8_t sum = 0;
  bool is_frame_closed = true;
  int _carrierHz = 0;
};

/**
 * @brief ESP32-specific TxDriver implementation that uses TxProtocolESP32 for
 * transmission.
 */
class TxDriverESP32 : public TxDriverCommon {
 public:
  /**
   * @param codec IR codec
   * @param pin TX pin
   * @param carrierHz Carrier frequency
   * @param freqHz Bit frequency
   * @param duty Duty cycle
   * @param useChecksum If true, append checksum to frame (default: false)
   */
  TxDriverESP32(Codec& codec, uint8_t pin, uint16_t freq = CARRIER_HZ,
                bool useChecksum = false) {
    init(codec, pin, useChecksum);
  }

  void init(Codec& codec, uint8_t pin, uint8_t duty = 33,
            bool useChecksum = false) {
    TxDriverCommon::init(protocol, codec, pin, useChecksum);
    protocol.setCarrierHz(_carrierHz);
  }

 protected:
  TxProtocolESP32 protocol;
  uint16_t _carrierHz = CARRIER_HZ;

  void sendPreamble() { protocol.sendPreamble(); }

  void sendData(const uint8_t* data, uint8_t len) {
    protocol.sendData(data, len);
  }

  void sendEnd() { protocol.sendEnd(_useChecksum, true); }
};

}  // namespace pulsewire
