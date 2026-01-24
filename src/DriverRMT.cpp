// Copyright 2025 Pavel Suprunov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Created by jadjer on 24.01.2026.
//

#include "iebus/DriverRMT.hpp"

#include <algorithm>
#include <driver/gpio.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <ranges>

namespace iebus {

namespace {

auto constexpr BUFFER_SIZE   = 64;
auto constexpr RESOLUTION_HZ = 1'000'000;

auto constexpr FREE_BUS_INTERVAL_US = 500;

auto constexpr DATA_BIT_TOTAL_US     = 40;
Time constexpr DATA_BIT_0_HIGH_US    = 34;
Time constexpr DATA_BIT_1_HIGH_US    = 20;
Time constexpr DATA_BIT_THRESHOLD_US = ((DATA_BIT_0_HIGH_US - DATA_BIT_1_HIGH_US) / 2);

auto constexpr START_BIT_TOTAL_US     = 190;
Time constexpr START_BIT_HIGH_US      = 170;
Time constexpr START_BIT_THRESHOLD_US = ((START_BIT_TOTAL_US + DATA_BIT_TOTAL_US) / 2);

template <typename T> auto constexpr inRange(T const a, T const b, T const threshold) -> bool {
  auto const [min, max] = std::minmax(a, b);
  auto const isValid    = ((max - min) <= threshold);

  return isValid;
}

auto constexpr decodePulseToBit(Time const pulseWidth) noexcept -> std::optional<Bit> {
  if (inRange(pulseWidth, DATA_BIT_0_HIGH_US, DATA_BIT_THRESHOLD_US)) return 0;
  if (inRange(pulseWidth, DATA_BIT_1_HIGH_US, DATA_BIT_THRESHOLD_US)) return 1;

  return std::nullopt;
}

auto receiverDoneCallback(rmt_channel_handle_t receiveChannel, rmt_rx_done_event_data_t const* eventData, void* userContext) -> bool {
  auto const queue = static_cast<QueueHandle_t>(userContext);

  BaseType_t highTaskWakeup = pdFALSE;

  xQueueSendFromISR(queue, eventData, &highTaskWakeup);

  auto const isValid = (highTaskWakeup == pdTRUE);

  return isValid;
}

} // namespace

DriverRMT::DriverRMT(Pin const rx, Pin const tx, Pin enable) noexcept : m_rxPin(rx), m_txPin(tx), m_enablePin(enable) {
  m_queueHandler = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));

  gpio_config_t const outputConfiguration = {
      .pin_bit_mask = (1ULL << m_enablePin),
      .mode         = GPIO_MODE_OUTPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&outputConfiguration);

  rmt_rx_channel_config_t const receiverChannelConfiguration = {
      .gpio_num          = static_cast<gpio_num_t>(m_rxPin),
      .clk_src           = RMT_CLK_SRC_DEFAULT,
      .resolution_hz     = RESOLUTION_HZ,
      .mem_block_symbols = BUFFER_SIZE,
      .intr_priority     = 0,
      .flags             = {.invert_in = false, .with_dma = false, .allow_pd = false},
  };
  ESP_ERROR_CHECK(rmt_new_rx_channel(&receiverChannelConfiguration, &m_receiverChannel));

  rmt_rx_event_callbacks_t receiverEventCallbacks = {
      .on_recv_done = receiverDoneCallback,
  };
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(m_receiverChannel, &receiverEventCallbacks, m_queueHandler));

  rmt_tx_channel_config_t const transceiverChannelConfiguration = {
      .gpio_num          = static_cast<gpio_num_t>(m_txPin),
      .clk_src           = RMT_CLK_SRC_DEFAULT,
      .resolution_hz     = RESOLUTION_HZ,
      .mem_block_symbols = BUFFER_SIZE,
      .trans_queue_depth = 1,
      .intr_priority     = 0,
      .flags             = {.invert_out = false, .with_dma = false, .allow_pd = false, .init_level = 0},
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&transceiverChannelConfiguration, &m_transceiverChannel));

  ESP_ERROR_CHECK(rmt_enable(m_receiverChannel));
}

DriverRMT::~DriverRMT() noexcept {
  ESP_ERROR_CHECK(rmt_disable(m_receiverChannel));
  ESP_ERROR_CHECK(rmt_disable(m_transceiverChannel));

  ESP_ERROR_CHECK(rmt_del_channel(m_receiverChannel));
  ESP_ERROR_CHECK(rmt_del_channel(m_transceiverChannel));
}

auto DriverRMT::isEnabled() const noexcept -> bool {
  return m_enable;
}

auto DriverRMT::isBusHigh() const noexcept -> bool {
  return false;
}

auto DriverRMT::isBusLow() const noexcept -> bool {
  return false;
}

auto DriverRMT::isBusFree() const noexcept -> bool {
  return false;
}

auto DriverRMT::enable() noexcept -> void {
  m_enable = true;

  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), m_enable);
}

auto DriverRMT::disable() noexcept -> void {
  m_enable = false;

  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), m_enable);
}

auto DriverRMT::readStartBit() noexcept -> bool {
  auto const optionalPulseWidth = readPulseWidth(START_BIT_TOTAL_US);
  if (not optionalPulseWidth.has_value()) {
    return false;
  }
  auto const pulseWidth = optionalPulseWidth.value();

  auto const isStartBit = inRange(pulseWidth, START_BIT_HIGH_US, START_BIT_THRESHOLD_US);

  return isStartBit;
}

auto DriverRMT::readBits(Size const numBits) noexcept -> std::optional<Data> {
  Data result = 0;

  for ([[maybe_unused]] auto const _ : std::views::repeat(0, numBits)) {
    auto const optionalPulseWidth = readPulseWidth(DATA_BIT_TOTAL_US);
    if (not optionalPulseWidth.has_value()) {
      return std::nullopt;
    }
    auto const pulseWidth = optionalPulseWidth.value();

    auto const optionalBit = decodePulseToBit(pulseWidth);
    if (not optionalBit.has_value()) {
      return std::nullopt;
    }
    auto const bit = optionalBit.value();

    result = ((result << 1) | (bit & 1));
  }

  return result;
}

auto DriverRMT::readPulseWidth(FrameWidth const timeout) noexcept -> std::optional<Time> {
  rmt_receive_config_t const receiveConfiguration = {
      .signal_range_min_ns = 1'000,
      .signal_range_max_ns = 1'000 * timeout,
      .flags               = {.en_partial_rx = true},
  };

  rmt_symbol_word_t buffer[BUFFER_SIZE];
  if (rmt_receive(m_receiverChannel, buffer, sizeof(buffer), &receiveConfiguration) != ESP_OK) {
    return std::nullopt;
  }

  rmt_rx_done_event_data_t receiveData;
  if (xQueueReceive(m_queueHandler, &receiveData, portMAX_DELAY) != pdPASS) {
    return std::nullopt;
  }

  auto const frame = receiveData.received_symbols[0];

  if (frame.level0) {
    return frame.duration0;
  }

  if (frame.level1) {
    return frame.duration1;
  }

  return std::nullopt;
}

auto DriverRMT::writeStartBit() noexcept -> void {
}

auto DriverRMT::writeBits(Data data, Size numBits) noexcept -> void {
}

auto DriverRMT::writePulseWidth(Time pulse, Time frame) noexcept -> void {
}

} // namespace iebus
