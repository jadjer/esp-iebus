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
      .pin_bit_mask = ((1ULL << m_txPin) | (1ULL << m_enablePin) | (1ULL << GPIO_NUM_36) | (1ULL << GPIO_NUM_37)),
      .mode         = GPIO_MODE_OUTPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&outputConfiguration);
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

auto Driver::enableTest() const noexcept -> void {
  gpio_set_level(GPIO_NUM_36, 1);
}

auto Driver::disableTest() const noexcept -> void {
  gpio_set_level(GPIO_NUM_36, 0);
}

auto IRAM_ATTR Driver::readStartBit() const noexcept -> bool {
  auto const optionalPulseWidth = readPulseWidth(FREE_BUS_INTERVAL_US, START_BIT_TOTAL_US);
  if (not optionalPulseWidth.has_value()) {
    return false;
  }

  auto const pulseWidth = optionalPulseWidth.value();

  if (pulseWidth > START_BIT_TOTAL_US) {
    return false;
  }

  auto const isStarted = static_cast<bool>(pulseWidth > DATA_BIT_TOTAL_US);

  if (isStarted) {

  }

  return isStarted;
}

auto IRAM_ATTR Driver::readBit() const noexcept -> std::optional<Bit> {
  auto const optionalPulseWidth = readPulseWidth(DATA_BIT_TOTAL_US, DATA_BIT_TOTAL_US);

  if (not optionalPulseWidth.has_value()) {
    return std::nullopt;
  }

  auto const pulseWidth = optionalPulseWidth.value();

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

    if (auto const isWritten = writeBit(ackBit); not isWritten) {
      return std::nullopt;
    }

  } else {
    std::ignore = readBit();
  }

  if (not isValid) {
    return std::nullopt;
  }

  return result;
}

auto IRAM_ATTR Driver::readPulseWidth(Timer::Time const waitTimeoutUS, Timer::Time const pulseTimeoutUS) const noexcept -> std::optional<Timer::Time> {
  if (isBusLow()) {
    while (isBusLow()) {
      if (m_lowTimer.getTimeUS() > waitTimeoutUS) {
        return std::nullopt;
      }
    }
  }

  m_highTimer.reset();

  while (isBusHigh()) {
    if (m_highTimer.getTimeUS() > pulseTimeoutUS) {
      return std::nullopt;
    }
  }

  auto const pulseWidthUS = m_highTimer.getTimeUS();

  m_lowTimer.reset();

  return pulseWidthUS;
}

auto IRAM_ATTR Driver::writeStartBit() const noexcept -> bool {
  return writePulseWidth(FREE_BUS_INTERVAL_US, START_BIT_HIGH_US, START_BIT_TOTAL_US);
}

auto IRAM_ATTR Driver::writeBit(Bit const bit) const noexcept -> bool {
  auto const pulseWidth = ((bit & 1) ? DATA_BIT_1_HIGH_US : DATA_BIT_0_HIGH_US);

  return writePulseWidth(DATA_BIT_TOTAL_US, pulseWidth, DATA_BIT_TOTAL_US);
}

auto IRAM_ATTR Driver::writeField(Data const data, Size const numBits, std::optional<bool> const optionalAck) const noexcept -> bool {
  Bit parity = 0;

  for (Size i = 0; i < numBits; ++i) {
    auto const bitPosition = (numBits - 1 - i);
    auto const bit         = static_cast<Bit>((data >> bitPosition) & 1);

    parity ^= bit;

    if (auto const isWritten = writeBit(bit); not isWritten) {
      return false;
    }
  }

  if (auto const isWritten = writeBit(parity); not isWritten) {
    return false;
  }

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

      if (auto const isWritten = writeBit(ackBit); not isWritten) {
        return false;
      }
    }
  }

  return true;
}

auto IRAM_ATTR Driver::writePulseWidth(Timer::Time const waitTimeoutUS, Timer::Time const pulseUS, Timer::Time const frameUS) const noexcept -> bool {
  if (isBusHigh()) {
    m_highTimer.reset();

    while (isBusHigh()) {
      if (m_highTimer.getTimeUS() > waitTimeoutUS) {
        return false;
      }
    }
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
