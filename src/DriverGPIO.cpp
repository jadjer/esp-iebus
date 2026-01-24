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

#include "iebus/DriverGPIO.hpp"

#include <algorithm>
#include <driver/gpio.h>
#include <ranges>

namespace iebus {

namespace {

Time constexpr FREE_BUS_INTERVAL_US = 500;

Time constexpr DATA_BIT_TOTAL_US     = 40;
Time constexpr DATA_BIT_0_HIGH_US    = 34;
Time constexpr DATA_BIT_1_HIGH_US    = 20;
Time constexpr DATA_BIT_THRESHOLD_US = ((DATA_BIT_0_HIGH_US - DATA_BIT_1_HIGH_US) / 2);

Time constexpr START_BIT_TOTAL_US     = 190;
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

} // namespace

DriverGPIO::DriverGPIO(Pin const rx, Pin const tx, Pin const enable) noexcept : m_rxPin(rx), m_txPin(tx), m_enablePin(enable) {
  gpio_config_t const inputConfiguration = {
      .pin_bit_mask = (1ULL << m_rxPin),
      .mode         = GPIO_MODE_INPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_NEGEDGE,
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

  m_timer.start();
  m_busFreeTimer.start();
}

DriverGPIO::~DriverGPIO() noexcept {
  gpio_glitch_filter_disable(m_filter);
  gpio_del_glitch_filter(m_filter);

  m_timer.stop();
  m_busFreeTimer.stop();
}

auto DriverGPIO::isEnabled() const noexcept -> bool {
  return m_enable;
}

auto DriverGPIO::isBusHigh() const noexcept -> bool {
  return (gpio_get_level(static_cast<gpio_num_t>(m_rxPin)) == 1);
}

auto DriverGPIO::isBusLow() const noexcept -> bool {
  return (gpio_get_level(static_cast<gpio_num_t>(m_rxPin)) == 0);
}

auto DriverGPIO::isBusFree() const noexcept -> bool {
  if (isBusHigh()) {
    return false;
  }

  auto const freeTime = m_busFreeTimer.getTime();
  auto const isFree   = (freeTime >= FREE_BUS_INTERVAL_US);

  return isFree;
}

auto DriverGPIO::enable() noexcept -> void {
  m_enable = true;

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), m_enable);
}

auto DriverGPIO::disable() noexcept -> void {
  m_enable = false;

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), m_enable);
}

auto DriverGPIO::readStartBit() noexcept -> bool {
  auto const optionalPulseWidth = readPulseWidth(START_BIT_TOTAL_US);
  if (not optionalPulseWidth.has_value()) {
    return false;
  }
  auto const pulseWidth = optionalPulseWidth.value();

  auto const isStartBit = inRange(pulseWidth, START_BIT_HIGH_US, START_BIT_THRESHOLD_US);

  return isStartBit;
}

auto DriverGPIO::readBits(Size const numBits) noexcept -> std::optional<Data> {
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

auto DriverGPIO::readPulseWidth(Time const timeout) noexcept -> std::optional<Time> {
  while (isBusLow()) {}

  m_timer.reset();

  while (isBusHigh()) {
    auto const periodDelay = m_timer.getTime();
    if (periodDelay > timeout) {
      return std::nullopt;
    }
  }

  auto const pulseWidth = m_timer.getTime();

  m_busFreeTimer.reset();

  return pulseWidth;
}

auto DriverGPIO::writeStartBit() noexcept -> void {
  writePulseWidth(START_BIT_HIGH_US, START_BIT_TOTAL_US);
}

auto DriverGPIO::writeBits(Data const data, Size const numBits) noexcept -> void {
  for (auto const bitPosition : std::views::iota(static_cast<Size>(0), numBits) | std::views::reverse) {
    auto const bit        = static_cast<Bit>((data >> bitPosition) & 1);
    auto const pulseWidth = (bit ? DATA_BIT_1_HIGH_US : DATA_BIT_0_HIGH_US);

    writePulseWidth(pulseWidth, DATA_BIT_TOTAL_US);
  }
}

auto DriverGPIO::writePulseWidth(Time const pulse, Time const frame) noexcept -> void {
  m_timer.reset();

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 1);
  while (m_timer.getTime() < pulse) {}

  m_busFreeTimer.reset();

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
  while (m_timer.getTime() < frame) {}
}

} // namespace iebus
