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

Timer::Time constexpr START_BIT_HIGH_US  = 171;
Timer::Time constexpr START_BIT_WAIT_US  = 7;
Timer::Time constexpr START_BIT_TOTAL_US = 193;

Timer::Time constexpr DATA_BIT_TOTAL_US   = 40;
Timer::Time constexpr DATA_BIT_1_HIGH_US  = 20;
Timer::Time constexpr DATA_BIT_0_HIGH_US  = 33;
Timer::Time constexpr DATA_BIT_NEUTRAL_US = ((DATA_BIT_0_HIGH_US + DATA_BIT_1_HIGH_US + 1) / 2);

Timer::Time constexpr PROCESS_COMPENSATION      = 3;
Timer::Time constexpr MESSAGE_SAFE_INTERVALE_US = (DATA_BIT_TOTAL_US * 10);

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
      .pin_bit_mask = ((1ULL << m_txPin)),
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
  return (readLevel() == 0);
}

auto IRAM_ATTR Driver::isBusHigh() const noexcept -> bool {
  return (readLevel() == 1);
}

auto IRAM_ATTR Driver::isBusFree() const noexcept -> bool {
  if (isBusHigh()) return false;

  return (m_lowTimer.getTime() > MESSAGE_SAFE_INTERVALE_US);
}

auto IRAM_ATTR Driver::enable() const noexcept -> void {
  writeLevel(0);
  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), 1);
}

auto IRAM_ATTR Driver::disable() const noexcept -> void {
  writeLevel(0);
  gpio_set_level(static_cast<gpio_num_t>(m_enablePin), 0);
}

auto IRAM_ATTR Driver::readStartBit() noexcept -> bool {
  auto const optPulseWidth = readPulseWidth(START_BIT_WAIT_US, START_BIT_TOTAL_US);
  if (not optPulseWidth) return false;

  return ((*optPulseWidth) > DATA_BIT_TOTAL_US);
}

auto IRAM_ATTR Driver::readBit() noexcept -> Driver::OptBit {
  auto const optPulseWidth = readPulseWidth(DATA_BIT_TOTAL_US, DATA_BIT_TOTAL_US);
  if (not optPulseWidth) return std::nullopt;

  return (((*optPulseWidth) < DATA_BIT_NEUTRAL_US) ? 1 : 0);
}

auto IRAM_ATTR Driver::readField(Size const numBits, Driver::OptBool const optAck, Driver::AddressList const addressList) noexcept -> Driver::OptData {
  Bit parity            = 0;
  Data field            = 0;
  bool isParityValid    = true;
  bool isAddressMatched = addressList.empty();

  for (Size i = 0; i < numBits; ++i) {
    auto const optBit = readBit();
    if (not optBit) return std::nullopt;

    auto const bit = (*optBit);

    field = ((field << 1) | bit);
    parity ^= bit;
  }

  {
    auto const optParityBit = readBit();
    if (not optParityBit) return std::nullopt;
    if ((*optParityBit) != parity) isParityValid = false;
  }

  for (auto const address : addressList) {
    if (address == field) isAddressMatched = true;
  }

  if (optAck) {
    auto const forDevice = (*optAck);
    if (forDevice and isAddressMatched) {
      if (not writeAck(isParityValid ? AckType::ACK : AckType::NAK)) return std::nullopt;
    } else {
      if (not readBit()) return std::nullopt;
    }
  }

  if (not isParityValid) return std::nullopt;

  return field;
}

auto IRAM_ATTR Driver::readAck() noexcept -> AckType {
  writeFrame(DATA_BIT_1_HIGH_US, DATA_BIT_NEUTRAL_US);

  auto const busLevel = readLevel();

  while (m_highTimer.getTime() < DATA_BIT_TOTAL_US) {}

  return ((busLevel == 1) ? AckType::ACK : AckType::NAK);
}

auto IRAM_ATTR Driver::readPulseWidth(Timer::Time const waitPulseTimeout, Timer::Time const pulseWidthTimeout) noexcept -> Driver::OptTime {
  if (not waitLevel(1, waitPulseTimeout)) return std::nullopt;

  m_highTimer.reset();

  if (not waitLevel(0, pulseWidthTimeout)) return std::nullopt;

  auto const pulseWidth = m_highTimer.getTime();

  m_lowTimer.reset();

  return pulseWidth;
}

auto IRAM_ATTR Driver::readLevel() const -> Bit {
  return gpio_get_level(static_cast<gpio_num_t>(m_rxPin));
}

auto IRAM_ATTR Driver::writeStartBit() noexcept -> bool {
  writeFrame(START_BIT_HIGH_US, START_BIT_TOTAL_US);

  return isBusLow();
}

auto IRAM_ATTR Driver::writeBit(Bit const bit) noexcept -> bool {
  auto const high = (bit ? DATA_BIT_1_HIGH_US : DATA_BIT_0_HIGH_US);

  writeFrame(high, DATA_BIT_TOTAL_US);

  return isBusLow();
}

auto IRAM_ATTR Driver::writeField(Data const data, Size const numBits, Driver::OptBool const optAck) noexcept -> bool {
  Bit parity = 0;

  for (Size i = 0; i < numBits; ++i) {
    auto const bit = static_cast<Bit>((data >> (numBits - 1 - i)) & 1);

    if (not writeBit(bit)) return false;

    parity ^= bit;
  }

  if (not writeBit(parity)) return false;

  if (optAck) {
    auto const forDevice = (*optAck);
    if (forDevice) {
      if (readAck() == AckType::NAK) return false;
    } else {
      if (not writeBit(static_cast<Bit>(AckType::NAK))) return false;
    }
  }

  return true;
}

auto IRAM_ATTR Driver::writeAck(AckType const ackType) -> bool {
  auto const high = ((ackType == AckType::ACK) ? DATA_BIT_0_HIGH_US : DATA_BIT_1_HIGH_US);

  if (not waitLevel(1, DATA_BIT_TOTAL_US)) return false;

  writeFrame((high - PROCESS_COMPENSATION), (DATA_BIT_TOTAL_US - PROCESS_COMPENSATION));

  return true;
}

auto IRAM_ATTR Driver::writeFrame(Timer::Time const pulse, Timer::Time const frame) noexcept -> void {
  m_highTimer.reset();

  writeLevel(1);
  while (m_highTimer.getTime() < pulse) {}

  m_lowTimer.reset();

  writeLevel(0);
  while (m_highTimer.getTime() < frame) {}
}

auto IRAM_ATTR Driver::writeLevel(Bit const bit) const -> void {
  gpio_set_level(static_cast<gpio_num_t>(m_txPin), bit);
}

auto IRAM_ATTR Driver::waitLevel(Bit const targetLevel, Timer::Time const timeoutUS) -> bool {
  if (readLevel() == targetLevel) return true;

  m_waitTimer.reset();

  while (readLevel() != targetLevel) {
    if (m_waitTimer.getTime() > timeoutUS) return false;
  }

  return true;
}

} // namespace iebus
