
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

  void setCarrierHz(uint32_t carrierHz) { _carrierHz = carrierHz; }

  bool begin(uint32_t bitFrequencyHz, Codec* p_codec, uint8_t pin) {
    Logger::info("begin TxProtocolESP32 with bitFrequencyHz=%d, pin=%d",
                 bitFrequencyHz, pin);
    this->_codec = p_codec;
    this->_txPin = pin;
    if (!initCodec(bitFrequencyHz)) return false;

    rmt_channel_handle_t tx_channel = nullptr;
    if (!createTxChannel(tx_channel)) return false;
    if (!applyCarrierConfig(tx_channel)) return false;
    if (!createCopyEncoder()) return false;
    if (!enableTxChannel(tx_channel)) return false;

    _txChannel = tx_channel;
    _txTransmitConfig = {.loop_count = 0, .flags = {}};
    return true;
  }

  void sendPreamble() override {
    if (is_frame_closed) {
      sum = 0;
      output.clear();
      // Insert preamble edges at the start
      _codec->getPreamble().getEdges(output);
      is_frame_closed = false;
      for (const auto& edge : output) {
        Logger::debug("Preamble edge: level=%d, duration=%d us", edge.level,
                      edge.pulseUs);
      }
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
    // Ensure even number of edges - pad with minimal edge if needed
    if (output.size() % 2 != 0) {
      Logger::debug("Odd edge count %zu, padding with 1us edge", output.size());
      if (output.size() > 0) {
        output.push_back(OutputEdge(!output.back().level, 1));
      }
    }
    size_t symbolCount = output.size() / 2;

    if (_itemsBuffer.size() < symbolCount) _itemsBuffer.resize(symbolCount);
    for (size_t i = 0, j = 0; i + 1 < output.size(); i += 2, ++j) {
      _itemsBuffer[j].duration0 = output[i].pulseUs;
      _itemsBuffer[j].level0 = output[i].level ? 1 : 0;
      _itemsBuffer[j].duration1 = output[i + 1].pulseUs;
      _itemsBuffer[j].level1 = output[i + 1].level ? 1 : 0;
      Logger::debug(
          "TX edge %zu: level0=%d, duration0=%d us, level1=%d, duration1=%d "
          "us",
          j, output[i].level, output[i].pulseUs, output[i + 1].level,
          output[i + 1].pulseUs);
    }
    if (_txChannel && _txEncoder && symbolCount > 0) {
      esp_err_t err = rmt_transmit(_txChannel, _txEncoder, _itemsBuffer.data(),
                                   symbolCount * sizeof(rmt_symbol_word_t),
                                   &_txTransmitConfig);
      if (err != ESP_OK) {
        Logger::error("RMT tx failed: %d", err);
      } else {
        rmt_tx_wait_all_done(_txChannel, portMAX_DELAY);
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

    // add delay: create idle period after frame
    if (isDelayAfterFrame) {
      uint32_t delayUs = _codec->getEndOfFrameDelayUs();
      bool idleLevel = _codec->getIdleLevel();
      // Ensure the frame ends in idle state with proper delay
      output.push_back(OutputEdge(idleLevel, delayUs));
      output.push_back(
          OutputEdge(idleLevel, 1));  // Minimal opposite edge to close symbol
    }
    // Convert OutputSpec to RMT symbols
    // Ensure even number of edges - pad with minimal edge if needed
    if (output.size() % 2 != 0) {
      Logger::error("Odd edge count %zu in sendEnd, padding with 1us edge",
                    output.size());
      if (output.size() > 0) {
        output.push_back(OutputEdge(!output.back().level, 1));
      }
    }
    size_t symbolCount = output.size() / 2;
    if (_itemsBuffer.size() < symbolCount) _itemsBuffer.resize(symbolCount);
    for (size_t i = 0, j = 0; i + 1 < output.size(); i += 2, ++j) {
      _itemsBuffer[j].duration0 = output[i].pulseUs;
      _itemsBuffer[j].level0 = output[i].level ? 1 : 0;
      _itemsBuffer[j].duration1 = output[i + 1].pulseUs;
      _itemsBuffer[j].level1 = output[i + 1].level ? 1 : 0;

      Logger::debug(
          "TX End frame edge %zu: level0=%d, duration0=%d us, level1=%d, "
          "duration1=%d us",
          j, output[i].level, output[i].pulseUs, output[i + 1].level,
          output[i + 1].pulseUs);
    }
    if (_txChannel && _txEncoder && symbolCount > 0) {
      esp_err_t err = rmt_transmit(_txChannel, _txEncoder, _itemsBuffer.data(),
                                   symbolCount * sizeof(rmt_symbol_word_t),
                                   &_txTransmitConfig);
      if (err != ESP_OK) {
        Logger::error("RMT tx failed: %d", err);
      } else {
        rmt_tx_wait_all_done(_txChannel, portMAX_DELAY);
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
  uint32_t _carrierHz = 0;

 private:
  static constexpr uint32_t kRmtResolutionHz = 1000000;

  bool initCodec(uint32_t bitFrequencyHz) {
    if (!_codec->begin(bitFrequencyHz)) {
      Logger::error("Codec initialization failed");
      return false;
    }
    return true;
  }

  bool createTxChannel(rmt_channel_handle_t& tx_channel) {
    uint8_t dma = (_frameSize > 64) ? 1 : 0;
    uint16_t memBlockSymbols = 64;  // ESP-IDF requires even and >= 64
    if (memBlockSymbols & 0x1) ++memBlockSymbols;

    rmt_tx_channel_config_t tx_config = {.gpio_num = (gpio_num_t)_txPin,
                                         .clk_src = RMT_CLK_SRC_DEFAULT,
                                         .resolution_hz = kRmtResolutionHz,
                                         .mem_block_symbols = memBlockSymbols,
                                         .trans_queue_depth = 4,
                                         .flags = {
                                             .invert_out = 0,
                                             .with_dma = dma,
                                         }};

    esp_err_t tx_result = rmt_new_tx_channel(&tx_config, &tx_channel);
    if (tx_result != ESP_OK && dma) {
      Logger::error("RMT TX DMA unsupported, retrying without DMA");
      tx_config.flags.with_dma = 0;
      tx_result = rmt_new_tx_channel(&tx_config, &tx_channel);
    }
    if (tx_result != ESP_OK || tx_channel == nullptr) {
      Logger::error("RMT TX channel initialization failed: %d", tx_result);
      return false;
    }
    return true;
  }

  bool applyCarrierConfig(rmt_channel_handle_t tx_channel) {
    if (_carrierHz == 0) return true;

    uint32_t carrierHz = _carrierHz;
    uint32_t periodTicks = kRmtResolutionHz / carrierHz;
    if (periodTicks < 2) {
      carrierHz = kRmtResolutionHz / 2;
      Logger::error(
          "Carrier frequency too high for RMT resolution, clamped to %u Hz",
          carrierHz);
    }

    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = carrierHz,  // e.g. 38000 for 38kHz
        .duty_cycle = 0.33f,        // range is 0.0 .. 1.0
    };
    esp_err_t carrier_result = rmt_apply_carrier(tx_channel, &carrier_cfg);
    if (carrier_result != ESP_OK) {
      Logger::error("RMT carrier config failed: %d", carrier_result);
      return false;
    }
    return true;
  }

  bool createCopyEncoder() {
    rmt_copy_encoder_config_t encoder_config = {};
    rmt_encoder_handle_t encoder = nullptr;
    esp_err_t enc_result = rmt_new_copy_encoder(&encoder_config, &encoder);
    if (enc_result != ESP_OK || encoder == nullptr) {
      Logger::error("RMT copy encoder init failed: %d", enc_result);
      return false;
    }
    _txEncoder = encoder;
    return true;
  }

  bool enableTxChannel(rmt_channel_handle_t tx_channel) {
    esp_err_t enable_result = rmt_enable(tx_channel);
    if (enable_result != ESP_OK) {
      Logger::error("RMT TX channel enable failed: %d", enable_result);
      return false;
    }
    return true;
  }
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
  TxDriverESP32(Codec& codec, uint8_t pin, uint32_t freq = CARRIER_HZ,
                bool useChecksum = false) {
    _carrierHz = freq;  // Store carrier frequency before init
    init(codec, pin, useChecksum);
  }

  void init(Codec& codec, uint8_t pin, uint8_t duty = 33,
            bool useChecksum = false) {
    TxDriverCommon::init(protocol, codec, pin, useChecksum);
    protocol.setCarrierHz(_carrierHz);
  }

 protected:
  TxProtocolESP32 protocol;
  uint32_t _carrierHz = CARRIER_HZ;

  void sendPreamble() { protocol.sendPreamble(); }

  void sendData(const uint8_t* data, uint8_t len) {
    protocol.sendData(data, len);
  }

  void sendEnd() { protocol.sendEnd(_useChecksum, true); }
};

}  // namespace pulsewire
