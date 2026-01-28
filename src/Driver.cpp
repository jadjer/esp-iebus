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

#include <algorithm>
#include <driver/gpio.h>
#include <esp_attr.h>
#include <limits>

namespace iebus {

namespace {

gpio_num_t constexpr INDICATOR_PIN = GPIO_NUM_36;

Timer::Time constexpr FREE_BUS_INTERVAL_US = 500;

Timer::Time constexpr DATA_BIT_TOTAL_US     = 40;
Timer::Time constexpr DATA_BIT_0_HIGH_US    = 34;
Timer::Time constexpr DATA_BIT_1_HIGH_US    = 20;
Timer::Time constexpr DATA_BIT_NEUTRAL_US   = ((DATA_BIT_0_HIGH_US + DATA_BIT_1_HIGH_US) / 2);
Timer::Time constexpr DATA_BIT_THRESHOLD_US = ((DATA_BIT_TOTAL_US - DATA_BIT_NEUTRAL_US) / 2);

Timer::Time constexpr START_BIT_TOTAL_US     = 190;
Timer::Time constexpr START_BIT_HIGH_US      = 170;
Timer::Time constexpr START_BIT_THRESHOLD_US = ((START_BIT_TOTAL_US - START_BIT_HIGH_US) / 2);

Timer::Time constexpr TIMEOUT_INFINITY_US = std::numeric_limits<std::uint64_t>::max();

template <typename T> auto constexpr inRange(T const a, T const b, T const threshold) -> bool {
  auto const [min, max] = std::minmax(a, b);
  auto const isValid    = ((max - min) <= threshold);

  return isValid;
}

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
      .pin_bit_mask = ((1ULL << m_txPin) | (1ULL << INDICATOR_PIN)),
      .mode         = GPIO_MODE_OUTPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&outputConfiguration);

  gpio_config_t const enableConfiguration = {
      .pin_bit_mask = (1ULL << m_enablePin),
      .mode         = GPIO_MODE_INPUT_OUTPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&enableConfiguration);
}

auto IRAM_ATTR Driver::isEnabled() const noexcept -> bool {
  return (gpio_get_level(static_cast<gpio_num_t>(m_enablePin)) == 1);
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

auto IRAM_ATTR Driver::enable() const noexcept -> void {
  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), 1);
}

auto IRAM_ATTR Driver::disable() const noexcept -> void {
  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), 0);
}

auto IRAM_ATTR Driver::startMessageIndicator() const noexcept -> void {
  gpio_set_level(INDICATOR_PIN, 1);
}

auto IRAM_ATTR Driver::stopMessageIndicator() const noexcept -> void {
  gpio_set_level(INDICATOR_PIN, 0);
}

auto IRAM_ATTR Driver::readStartBit() const noexcept -> bool {
  auto const optPulseWidth = readPulseWidth(TIMEOUT_INFINITY_US, START_BIT_TOTAL_US);
  if (not optPulseWidth) return false;

  auto const pulseWidth = (*optPulseWidth);

  return inRange(pulseWidth, START_BIT_HIGH_US, START_BIT_THRESHOLD_US);
}

auto IRAM_ATTR Driver::readBit() const noexcept -> Driver::OptBit {
  auto const optPulseWidth = readPulseWidth(DATA_BIT_TOTAL_US, DATA_BIT_TOTAL_US);
  if (not optPulseWidth) return std::nullopt;

  auto const pulseWidth = (*optPulseWidth);

  if (inRange(pulseWidth, DATA_BIT_0_HIGH_US, DATA_BIT_THRESHOLD_US)) return 0;
  if (inRange(pulseWidth, DATA_BIT_1_HIGH_US, DATA_BIT_THRESHOLD_US)) return 1;

  return std::nullopt;
}

auto IRAM_ATTR Driver::readField(Size const numBits, Driver::OptBool const optWriteAck, Driver::OptAddress const optAddress) const noexcept -> Driver::OptData {
  Address const address = optAddress.value_or(0);

  Bit parity            = 0;
  Data field            = 0;
  bool isParityValid    = true;
  bool isAddressMatched = true;

  for (Size i = 0; i < numBits; ++i) {
    auto const optBit = readBit();
    if (not optBit) return std::nullopt;

    auto const bit = (*optBit);

    field = ((field << 1) | bit);
    parity ^= bit;

    if (optAddress and isAddressMatched) {
      auto const addressBitPosition = (numBits - 1 - i);
      auto const addressBit         = static_cast<Bit>((address >> addressBitPosition) & 1);

      if (addressBit != bit) isAddressMatched = false;
    }
  }

  auto const optParityBit = readBit();
  if (not optParityBit) return std::nullopt;
  if ((*optParityBit) != parity) isParityValid = false;

  if (optWriteAck) {
    if ((*optWriteAck) and isAddressMatched) {
      while (m_highTimer.getTimeUS() < DATA_BIT_TOTAL_US) {}

      auto const ackBit = static_cast<Bit>(isParityValid ? AckType::ACK : AckType::NAK);
      if (not writeBit(ackBit)) return std::nullopt;

    } else {
      if (not readBit()) return std::nullopt;
    }
  }

  if (not isParityValid) return std::nullopt;

  return field;
}

auto IRAM_ATTR Driver::readPulseWidth(Timer::Time const waitTimeoutUS, Timer::Time const pulseTimeoutUS) const noexcept -> Driver::OptTime {
  while (isBusLow()) {
    if (m_lowTimer.getTimeUS() > waitTimeoutUS) return std::nullopt;
  }

  m_highTimer.reset();

  while (isBusHigh()) {
    if (m_highTimer.getTimeUS() > pulseTimeoutUS) return std::nullopt;
  }

  auto const pulseWidth = m_highTimer.getTimeUS();

  m_lowTimer.reset();

  return pulseWidth;
}

auto IRAM_ATTR Driver::writeStartBit() const noexcept -> bool {
  return writePulseWidth(TIMEOUT_INFINITY_US, START_BIT_HIGH_US, START_BIT_TOTAL_US);
}

auto IRAM_ATTR Driver::writeBit(Bit const bit) const noexcept -> bool {
  auto const pulseWidth = (bit ? DATA_BIT_1_HIGH_US : DATA_BIT_0_HIGH_US);

  return writePulseWidth(DATA_BIT_TOTAL_US, pulseWidth, DATA_BIT_TOTAL_US);
}

auto IRAM_ATTR Driver::writeField(Data const data, Size const numBits, Driver::OptBool const optReadAck) const noexcept -> bool {
  Bit parity = 0;

  for (Size i = 0; i < numBits; ++i) {
    auto const bitPosition = (numBits - 1 - i);
    auto const bit         = static_cast<Bit>((data >> bitPosition) & 1);

    parity ^= bit;

    if (not writeBit(bit)) return false;
  }

  if (writeBit(parity)) return false;

  if (optReadAck) {
    if (*optReadAck) {
      auto const optAckBit = readBit();
      if (not optAckBit) return false;

      auto const ack = static_cast<AckType>(*optAckBit);
      if (ack == AckType::NAK) return false;

    } else {
      auto const ackBit = static_cast<Bit>(AckType::NAK);
      if (writeBit(ackBit)) return false;
    }
  }

  return true;
}

auto IRAM_ATTR Driver::writePulseWidth(Timer::Time const waitTimeoutUS, Timer::Time const pulseUS, Timer::Time const frameUS) const noexcept -> bool {
  while (isBusHigh()) {
    if (m_highTimer.getTimeUS() > waitTimeoutUS) return false;
  }

  m_highTimer.reset();

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 1);

  while (m_highTimer.getTimeUS() < pulseUS) {}

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);

  m_lowTimer.reset();

  while (m_highTimer.getTimeUS() < frameUS) {}

  return true;
}

} // namespace iebus
