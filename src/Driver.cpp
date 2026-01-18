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

namespace iebus {

namespace {

auto constexpr START_BIT_TOTAL_US = 193;
auto constexpr START_BIT_HIGH_US = 171;

auto constexpr DATA_BIT_TOTAL_US  = 39;
auto constexpr DATA_BIT_0_HIGH_US = 33;
auto constexpr DATA_BIT_1_HIGH_US = 20;

auto constexpr pulseToBit(Time const pulseWidth) noexcept -> Bit {
  auto constexpr BIT_THRESHOLD_US = (DATA_BIT_0_HIGH_US + DATA_BIT_1_HIGH_US) / 2;
  return pulseWidth <= BIT_THRESHOLD_US;
}

} // namespace

Driver::Driver(Pin const rx, Pin const tx, Pin const enable) noexcept : m_rxPin(rx), m_txPin(tx), m_enablePin(enable), m_enable(false), m_timer() {
  gpio_config_t const inputConfiguration = {
      .pin_bit_mask = (1ULL << m_rxPin),
      .mode         = GPIO_MODE_INPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&inputConfiguration));

  gpio_config_t const outputConfiguration = {
      .pin_bit_mask = (1ULL << m_txPin) | (1ULL << m_enablePin),
      .mode         = GPIO_MODE_OUTPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&outputConfiguration));
}

auto Driver::isEnabled() const noexcept -> bool {
  return m_enable;
}

auto Driver::isBusHigh() const noexcept -> bool {
  return gpio_get_level(m_rxPin);
}

auto Driver::isBusLow() const noexcept -> bool {
  return not isBusHigh();
}

auto Driver::isBusFree() const noexcept -> bool {
  m_timer.reset();

  while (isBusLow()) {
    auto const delay = m_timer.getTime();
    if (delay > DATA_BIT_TOTAL_US) {
      return true;
    }
  }

  return false;
}

auto Driver::enable() noexcept -> void {
  ESP_ERROR_CHECK(gpio_set_level(m_txPin, 0));
  ESP_ERROR_CHECK(gpio_set_level(m_enablePin, 1));

  while (isBusHigh()) {}

  m_timer.enable();
  m_timer.start();

  m_enable = true;
}

auto Driver::disable() noexcept -> void {
  ESP_ERROR_CHECK(gpio_set_level(m_txPin, 0));
  ESP_ERROR_CHECK(gpio_set_level(m_enablePin, 0));

  m_timer.stop();
  m_timer.disable();

  m_enable = false;
}

auto Driver::readStartBit() const noexcept -> bool {
  auto constexpr START_BIT_THRESHOLD = 20;
  auto constexpr LOWER_LIMIT         = START_BIT_HIGH_US - START_BIT_THRESHOLD;
  auto constexpr UPPER_LIMIT         = START_BIT_HIGH_US + START_BIT_THRESHOLD;

  auto const pulseWidth = readPulseWidth();
  return (pulseWidth >= LOWER_LIMIT and pulseWidth <= UPPER_LIMIT);
}

auto Driver::readBits(Size const numBits) const noexcept -> Data {
  Data result = 0;

  for (Size i = 0; i < numBits; ++i) {
    auto const pulseWidth = readPulseWidth();
    auto const bit        = pulseToBit(pulseWidth);

    result = (result << 1) | bit;
  }

  return result;
}

auto Driver::writeStartBit() const noexcept -> void {
  writePulseWidth(START_BIT_HIGH_US, START_BIT_TOTAL_US);
}

auto Driver::writeBits(Data const data, Size const numBits) const noexcept -> void {
  for (Size i = 0; i < numBits; ++i) {
    auto const bitPosition = numBits - 1 - i;
    auto const bit         = static_cast<Bit>(data >> bitPosition & 1);

    auto const highDuration = bit ? DATA_BIT_1_HIGH_US : DATA_BIT_0_HIGH_US;

    writePulseWidth(highDuration, DATA_BIT_TOTAL_US);
  }
}

auto Driver::readPulseWidth() const noexcept -> Time {
  while (isBusLow()) {}

  m_timer.reset();

  while (isBusHigh()) {}

  return m_timer.getTime();
}

auto Driver::writePulseWidth(Time const pulse, Time const frame) const noexcept -> void {
  m_timer.reset();

  ESP_ERROR_CHECK(gpio_set_level(m_txPin, 1));

  while (m_timer.getTime() < pulse) {}

  ESP_ERROR_CHECK(gpio_set_level(m_txPin, 0));

  while (m_timer.getTime() < frame) {}
}

} // namespace iebus
