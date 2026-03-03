#pragma once
#include <Arduino.h>
#include <PIO.h>
#include <RP2040.h>

#include "pulse/Codec.h"
#include "pulse/TxDriverCommon.h"
#include "pulse/TxProtocol.h"
#include "pulse/Vector.h"

namespace pulsewire {

/**
 * @brief RP2040-specific TxProtocol implementation using PIO and DMA for
 * precise timing.
 */
class TxProtocolRP2040 : public TxProtocol {
 public:
  TxProtocolRP2040() = default;

  void setFrameSize(uint16_t size) override {
    _frameSize = size;
    _frameBuffer.resize(size);
  }

  bool begin(Codec* p_codec, uint8_t pin) override {
    this->_codec = p_codec;
    this->_txPin = pin;

    if (!_codec->begin()) {
      Logger::error("Codec initialization failed");
      return false;
    }

    pinMode(_txPin, OUTPUT);

    // Load PIO program only once
    if (offset == 0) {
      offset = pio.add_program(ir_tx_pio_program);
      if (offset == 0) {
        Logger::error("PIO program loading failed");
        return false;
      }
    }

    // Configure PIO state machine
    bool pin_ok = pio.set_pin_direction(_txPin, true);
    bool sm_ok = pio.set_sm_config(sm, offset, _txPin);
    if (!pin_ok || !sm_ok) {
      Logger::error("PIO state machine configuration failed");
      return false;
    }

    return true;
  }

  void sendPreamble() override {
    if (is_frame_closed) {
      sum = 0;
      output.clear();
      _codec->getPreamble().getEdges(output);
      is_frame_closed = false;
    }
  }

  void sendData(const uint8_t* data, uint8_t len, uint32_t bitPeriod) override {
    if (len == 0) return;

    for (uint8_t i = 0; i < len; ++i) {
      uint8_t b = data[i];
      sum += b;
      _codec->encode(b, bitPeriod, output);
    }

    // process data in chunks to avoid large buffers
    sendEdgesPIO(output);
    output.clear();
  }

  void sendEnd(bool& useChecksum, bool isDelayAfterFrame,
               uint32_t bitPeriod) override {
    if (is_frame_closed) return;

    if (useChecksum) {
      _codec->encode(sum, bitPeriod, output);
      sum = 0;
    }

    if (isDelayAfterFrame) {
      output.push_back(OutputEdge(_codec->getEndOfFrameUs(), 0));
      output.push_back(OutputEdge(0, 0));
    }

    sendEdgesPIO(output);
    is_frame_closed = true;
  }

  bool isFrameClosed() const override { return is_frame_closed; }

 protected:
  Codec* _codec = nullptr;
  uint8_t _txPin;
  uint16_t _frameSize = 64;
  Vector<OutputEdge> output;
  uint8_t sum = 0;
  bool is_frame_closed = true;

  PIO pio = PIO(0);
  uint8_t sm = 0;
  uint32_t offset = 0;

  void sendEdgesPIO(const Vector<OutputEdge>& edges) {
    pio.sm_set_enabled(sm, true);

    Vector<uint32_t> txBuffer;
    for (const auto& edge : edges) {
      txBuffer.push_back(edge.pulseUs);
      txBuffer.push_back(edge.level ? 1 : 0);
    }

    // Setup DMA transfer
    int dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio.pio(), sm, true));

    dma_channel_configure(dma_channel, &c,
                          &pio.pio()->txf[sm],  // Write address (PIO TX FIFO)
                          txBuffer.data(),      // Read address (buffer)
                          txBuffer.size(),      // Number of transfers
                          true);                // Start immediately

    dma_channel_wait_for_finish_blocking(dma_channel);
    dma_channel_unclaim(dma_channel);
  }

  // PIO assembly for IR carrier output
  static const char* ir_tx_pio_program;
};

const char* TxProtocolRP2040::ir_tx_pio_program = R"(
  .program ir_tx
  pull block
  mov x, osr
  pull block
  mov y, osr
  jmp y mark
space:
  set pins, 0
  nop [31]
  jmp x-- space
  set pins, 0
  jmp ir_tx
mark:
  set pins, 1
  nop [carrier_half_period]
  set pins, 0
  nop [carrier_half_period]
  jmp x-- mark
  set pins, 0
  jmp ir_tx
)";

/**
 * @brief RP2040-specific TxDriver implementation that uses TxProtocolRP2040 for
 * transmission.
 */
class TxDriverRP2040 : public TxDriverCommon {
 public:
  TxDriverRP2040(Codec& codec, uint8_t pin, uint32_t carrierHz = CARRIER_HZ,
                 uint32_t freqHz = DEFAULT_BIT_FREQ_HZ,
                 bool useChecksum = false) {
    begin(codec, pin, freqHz, useChecksum);
  }

  bool begin(Codec& codec, uint8_t pin, uint32_t carrierHz = CARRIER_HZ,
             uint32_t freqHz = DEFAULT_BIT_FREQ_HZ, bool useChecksum = false) {
    if (!TxDriverCommon::begin(protocol, codec, pin, freqHz, useChecksum)) {
      Logger::error("TxDriverCommon initialization failed");
      return false;
    }
    if (!protocol.begin(&codec, pin)) {
      Logger::error("TxProtocolRP2040 initialization failed");
      return false;
    }
    return true;
  }

 protected:
  TxProtocolRP2040 protocol;

  void sendPreamble() { protocol.sendPreamble(); }

  void sendData(const uint8_t* data, uint8_t len) {
    protocol.sendData(data, len, _bitPeriod);
  }

  void sendEnd() { protocol.sendEnd(_useChecksum, true, _bitPeriod); }
};

}  // namespace pulsewire
