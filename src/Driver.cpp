// Copyright 2026 Pavel Suprunov
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
// Created by jadjer on 29.11.2025.
//

#include "iebus/Driver.hpp"

#include <driver/gpio.h>
#include <esp_attr.h>

namespace iebus {

namespace {

Timer::Time constexpr FREE_BUS_INTERVAL_US = 500;

Timer::Time constexpr DATA_BIT_TOTAL_US   = 40;
Timer::Time constexpr DATA_BIT_0_HIGH_US  = 33;
Timer::Time constexpr DATA_BIT_1_HIGH_US  = 20;
Timer::Time constexpr DATA_BIT_NEUTRAL_US = ((DATA_BIT_0_HIGH_US + DATA_BIT_1_HIGH_US) / 2);

Timer::Time constexpr START_BIT_TOTAL_US = 190;
Timer::Time constexpr START_BIT_HIGH_US  = 170;

} // namespace

Driver::Driver(Pin const rx, Pin const tx, Pin const enable) noexcept : m_rxPin(rx), m_txPin(tx), m_enablePin(enable) {
  gpio_config_t const inputConfiguration = {
      .pin_bit_mask = (1ULL << m_rxPin),
      .mode         = GPIO_MODE_INPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&inputConfiguration);

  gpio_config_t const outputConfiguration = {
      .pin_bit_mask = ((1ULL << m_txPin) | (1ULL << m_enablePin)),
      .mode         = GPIO_MODE_OUTPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&outputConfiguration);

  gpio_pin_glitch_filter_config_t const filterConfiguration = {
      .clk_src  = GLITCH_FILTER_CLK_SRC_DEFAULT,
      .gpio_num = static_cast<gpio_num_t>(m_rxPin),
  };
  gpio_new_pin_glitch_filter(&filterConfiguration, &m_filter);
  gpio_glitch_filter_enable(m_filter);
}

Driver::~Driver() noexcept {
  gpio_glitch_filter_disable(m_filter);
  gpio_del_glitch_filter(m_filter);
}

auto Driver::isEnabled() const noexcept -> bool {
  return m_enable;
}

auto IRAM_ATTR Driver::isBusLow() const noexcept -> bool {
  return (gpio_get_level(static_cast<gpio_num_t>(m_rxPin)) == 0);
}

auto IRAM_ATTR Driver::isBusHigh() const noexcept -> bool {
  return (gpio_get_level(static_cast<gpio_num_t>(m_rxPin)) == 1);
}

auto IRAM_ATTR Driver::isBusFree() const noexcept -> bool {
  if (isBusHigh()) return false;

  return (m_lowTimer.getTimeUS() >= FREE_BUS_INTERVAL_US);
}

auto Driver::enable() noexcept -> void {
  m_enable = true;

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), m_enable);
}

auto Driver::disable() noexcept -> void {
  m_enable = false;

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), m_enable);
}

auto IRAM_ATTR Driver::readStartBit() const noexcept -> bool {
  auto const pulseWidth = readPulseWidth();

  if (pulseWidth > START_BIT_TOTAL_US) {
    return false;
  }

  return static_cast<bool>(pulseWidth > DATA_BIT_TOTAL_US);
}

auto IRAM_ATTR Driver::readBit() const noexcept -> std::optional<Bit> {
  auto const pulseWidth = readPulseWidth();

  if (pulseWidth > DATA_BIT_TOTAL_US) {
    return std::nullopt;
  }

  return static_cast<Bit>(pulseWidth <= DATA_BIT_NEUTRAL_US);
}

auto IRAM_ATTR Driver::readField(Size const numBits, bool const sendAck, std::optional<Address> const optionalAddress) const noexcept -> std::optional<Data> {
  Bit parity     = 0;
  Data result    = 0;
  bool isValid   = true;
  bool isMatched = true;

  for (Size i = 0; i < numBits; ++i) {
    auto const optionalBit = readBit();
    if (not optionalBit.has_value()) {
      return std::nullopt;
    }

    auto const bit = optionalBit.value();

    result = ((result << 1) | bit);
    parity ^= bit;

    if (optionalAddress.has_value() and isMatched) {
      auto const address            = optionalAddress.value();
      auto const addressBitPosition = (numBits - 1 - i);
      auto const addressBit         = static_cast<Bit>((address >> addressBitPosition) & 1);

      if (bit != addressBit) {
        isMatched = false;
      }
    }
  }

  auto const optionalParityBit = readBit();
  if (not optionalParityBit.has_value()) {
    return std::nullopt;
  }

  auto const parityBit = optionalParityBit.value();

  if (parityBit != parity) {
    isValid = false;
  }

  if (sendAck and isMatched) {
    while (m_highTimer.getTimeUS() < DATA_BIT_TOTAL_US) {}

    auto const ack    = (isValid ? AckType::ACK : AckType::NAK);
    auto const ackBit = static_cast<Bit>(ack);

    writeBit(ackBit);

  } else {
    std::ignore = readBit();
  }

  if (not isValid) {
    return std::nullopt;
  }

  return result;
}

auto IRAM_ATTR Driver::readPulseWidth() const noexcept -> Timer::Time {
  while (isBusLow()) {}

  m_highTimer.reset();

  while (isBusHigh()) {}

  m_lowTimer.reset();

  return m_highTimer.getTimeUS();
}

auto IRAM_ATTR Driver::writeStartBit() const noexcept -> void {
  writePulseWidth(START_BIT_HIGH_US, START_BIT_TOTAL_US);
}

auto IRAM_ATTR Driver::writeBit(Bit const bit) const noexcept -> void {
  auto const pulseWidth = ((bit & 1) ? DATA_BIT_1_HIGH_US : DATA_BIT_0_HIGH_US);

  writePulseWidth(pulseWidth, DATA_BIT_TOTAL_US);
}

auto IRAM_ATTR Driver::writeField(Data const data, Size const numBits, std::optional<bool> const optionalAck) const noexcept -> bool {
  Bit parity = 0;

  for (Size i = 0; i < numBits; ++i) {
    auto const bitPosition = (numBits - 1 - i);
    auto const bit         = static_cast<Bit>((data >> bitPosition) & 1);

    parity ^= bit;

    writeBit(bit);
  }

  writeBit(parity);

  if (optionalAck.has_value()) {
    auto const forDevice = optionalAck.value();

    if (forDevice) {
      auto const optionalAckBit = readBit();
      if (not optionalAckBit.has_value()) {
        return false;
      }

      auto const ackBit = optionalAckBit.value();
      auto const ack    = static_cast<AckType>(ackBit);

      if (ack == AckType::NAK) {
        return false;
      }

    } else {
      auto const ack    = AckType::NAK;
      auto const ackBit = static_cast<Bit>(ack);

      writeBit(ackBit);
    }
  }

  return true;
}

auto IRAM_ATTR Driver::writePulseWidth(Timer::Time const pulseUS, Timer::Time const frameUS) const noexcept -> void {
  while (isBusHigh()) {}

  m_highTimer.reset();

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 1);
  while (m_highTimer.getTimeUS() < pulseUS) {}
  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);

  m_lowTimer.reset();

  while (m_highTimer.getTimeUS() < frameUS) {}
}

} // namespace iebus
