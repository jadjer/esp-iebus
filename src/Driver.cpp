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
// Created by jadjer on 29.11.2025.
//

#include "iebus/Driver.hpp"

#include <algorithm>

#include <driver/gpio.h>

#include "common.hpp"

namespace iebus {

namespace {

auto constexpr TAG = "IEBusDriver";

auto constexpr START_BIT_TOTAL_US = 190;
auto constexpr START_BIT_HIGH_US  = 171;
auto constexpr START_BIT_LOW_US   = START_BIT_TOTAL_US - START_BIT_HIGH_US;

auto constexpr DATA_BIT_TOTAL_US  = 39;
auto constexpr DATA_BIT_0_HIGH_US = 33;
auto constexpr DATA_BIT_0_LOW_US  = DATA_BIT_TOTAL_US - DATA_BIT_0_HIGH_US;
auto constexpr DATA_BIT_1_HIGH_US = 20;
auto constexpr DATA_BIT_1_LOW_US  = DATA_BIT_TOTAL_US - DATA_BIT_1_HIGH_US;

auto detectBitType(Time const bitDuration) -> Driver::BitType {
  auto const dStart = std::abs(bitDuration - START_BIT_HIGH_US);
  auto const d0     = std::abs(bitDuration - DATA_BIT_0_HIGH_US);
  auto const d1     = std::abs(bitDuration - DATA_BIT_1_HIGH_US);

  auto const minDiff = std::min({dStart, d0, d1});

  if (minDiff == dStart) {
    return Driver::BitType::BIT_START;
  }

  if (minDiff == d0) {
    return Driver::BitType::BIT_0;
  }

  if (minDiff == d1) {
    return Driver::BitType::BIT_1;
  }

  return Driver::BitType::BIN_UNKNOWN;
}

} // namespace

Driver::Driver(Driver::Pin const rx, Driver::Pin const tx, Driver::Pin const enable) noexcept : m_rxPin(rx), m_txPin(tx), m_enablePin(enable) {

  gpio_config_t const receiverConfiguration = {
      .pin_bit_mask = (1ULL << m_rxPin),
      .mode         = GPIO_MODE_INPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&receiverConfiguration);

  gpio_config_t const transmitterConfiguration = {
      .pin_bit_mask = (1ULL << m_txPin),
      .mode         = GPIO_MODE_OUTPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&transmitterConfiguration);

  gpio_config_t const enableConfiguration = {
      .pin_bit_mask = (1ULL << m_enablePin),
      .mode         = GPIO_MODE_OUTPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&enableConfiguration);
}

auto Driver::isEnabled() const -> bool {
  return m_isEnabled;
}

auto Driver::isBusHigh() const -> bool {
  return gpio_get_level(static_cast<gpio_num_t>(m_rxPin));
}

auto Driver::isBusLow() const -> bool {
  return not isBusHigh();
}

auto Driver::isBusFree() const -> bool {
  if (isBusHigh()) {
    return false;
  }

  auto const startTime = getTimeUS();

  while (isBusLow()) {
    auto const currentTime    = getTimeUS();
    auto const timeDifference = currentTime - startTime;
    if (timeDifference >= DATA_BIT_TOTAL_US) {
      return true;
    }
  }

  return false;
}

auto Driver::enable() -> void {
  m_isEnabled = true;

  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), 1);
}

auto Driver::disable() -> void {
  m_isEnabled = false;

  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), 0);
}

auto Driver::readStartBit() -> bool {
  waitBusHigh();

  auto const startTime = getTimeUS();

  waitBusLow();

  auto const stopTime    = getTimeUS();
  auto const bitDuration = stopTime - startTime;
  auto const bitType     = detectBitType(bitDuration);

  if (bitType == BitType::BIT_START) {
    return true;
  }

  return false;
}

auto Driver::readBit() -> std::optional<Driver::Bit> {
  waitBusHigh();

  auto const startTime = getTimeUS();

  waitBusLow();

  auto const stopTime    = getTimeUS();
  auto const bitDuration = stopTime - startTime;
  auto const bitType     = detectBitType(bitDuration);

  if (bitType == BitType::BIT_0) {
    return 0;
  }

  if (bitType == BitType::BIT_1) {
    return 1;
  }

  return std::nullopt;
}

auto Driver::readBits(Driver::Size const numBits) -> std::optional<Driver::Data> {
  Driver::Data result = 0;

  for (Driver::Size i = 0; i < numBits; ++i) {
    auto const optionalBit = readBit();
    if (not optionalBit) {
      return std::nullopt;
    }

    auto const bit      = optionalBit.value();
    auto const bitShift = numBits - 1 - i;

    result |= bit << bitShift;
  }

  return result;
}

auto Driver::readAckBit() -> std::optional<Driver::AckType> {
  auto const optionalBit = readBit();
  if (not optionalBit) {
    return std::nullopt;
  }

  auto const bit = optionalBit.value();

  if (bit == 0) {
    return Driver::AckType::ACK;
  }

  if (bit == 1) {
    return Driver::AckType::NAK;
  }

  return std::nullopt;
}

auto Driver::writeStartBit() const -> void {
  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 1);
  delayUS(START_BIT_HIGH_US);

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
  delayUS(START_BIT_LOW_US);
}

auto Driver::writeBit(Bit const bit) const -> void {
  auto const highDuration = bit ? DATA_BIT_1_HIGH_US : DATA_BIT_0_HIGH_US;
  auto const lowDuration  = bit ? DATA_BIT_1_LOW_US : DATA_BIT_0_LOW_US;

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 1);
  delayUS(highDuration);

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
  delayUS(lowDuration);
}

auto Driver::writeBits(Driver::Data const data, Driver::Size const numBits) const -> void {
  for (Driver::Size i = 0; i < numBits; ++i) {
    auto const bitPosition = numBits - 1 - i;
    auto const bit         = static_cast<Bit>(data >> bitPosition & 1);

    writeBit(bit);
  }
}

auto Driver::writeAckBit(Driver::AckType const ack) const -> void {
  if (ack == Driver::AckType::ACK) {
    return writeBit(0);
  }

  writeBit(1);
}

auto Driver::waitBusLow() -> void {
  while (isBusHigh()) {
    delayUS(1);
  }
}

auto Driver::waitBusHigh() -> void {
  while (isBusLow()) {
    delayUS(1);
  }
}

} // namespace iebus
