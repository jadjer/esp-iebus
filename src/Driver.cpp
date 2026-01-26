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
#include <esp_cpu.h>

namespace iebus {

namespace {

auto constexpr CPU_FREQUENCY_HZ = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;

Time constexpr FREE_BUS_INTERVAL_US = 500;

Time constexpr DATA_BIT_TOTAL_US       = 40;
Time constexpr DATA_BIT_0_HIGH_US      = 34;
Time constexpr DATA_BIT_1_HIGH_US      = 20;
Time constexpr DATA_BIT_NEUTRAL_US     = ((DATA_BIT_0_HIGH_US + DATA_BIT_1_HIGH_US) / 2);
Time constexpr DATA_BIT_TOTAL_CYCLES   = DATA_BIT_TOTAL_US * CPU_FREQUENCY_HZ;
Time constexpr DATA_BIT_NEUTRAL_CYCLES = DATA_BIT_NEUTRAL_US * CPU_FREQUENCY_HZ;

Time constexpr START_BIT_TOTAL_US   = 190;
Time constexpr START_BIT_HIGH_US    = 170;
Time constexpr START_BIT_NEUTRAL_US = ((START_BIT_TOTAL_US + DATA_BIT_TOTAL_US) / 2);

auto constexpr NO_OPERATION = [](auto&&...) {};

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
  auto constexpr FREE_BUS_INTERVAL_CYCLES = FREE_BUS_INTERVAL_US * CPU_FREQUENCY_HZ;

  while (isBusLow()) {
    auto const currentCycles   = esp_cpu_get_cycle_count();
    auto const differentCycles = (currentCycles - m_lowLevelStartCycles);

    if (differentCycles >= FREE_BUS_INTERVAL_CYCLES) {
      return true;
    }
  }

  return false;
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

auto IRAM_ATTR Driver::readStartBit() noexcept -> bool {
  auto constexpr START_BIT_TOTAL_CYCLES   = START_BIT_TOTAL_US * CPU_FREQUENCY_HZ;
  auto constexpr START_BIT_NEUTRAL_CYCLES = START_BIT_NEUTRAL_US * CPU_FREQUENCY_HZ;

  auto const optionalBit = readBit(START_BIT_NEUTRAL_CYCLES, START_BIT_TOTAL_CYCLES, NO_OPERATION);
  if (not optionalBit.has_value()) {
    return false;
  }

  auto const bit     = optionalBit.value();
  auto const isStart = static_cast<bool>(bit);

  return isStart;
}

auto IRAM_ATTR Driver::readDataBit() noexcept -> std::optional<Bit> {
  auto const optionalBit = readBit(DATA_BIT_NEUTRAL_CYCLES, DATA_BIT_TOTAL_CYCLES, NO_OPERATION);
  if (not optionalBit.has_value()) {
    return std::nullopt;
  }

  auto const bit = optionalBit.value();

  return bit;
}

auto IRAM_ATTR Driver::readField(Size const numBits, bool const sendAck, std::optional<Address> const optionalAddress) noexcept -> std::optional<Data> {
  Size const numBitsWithParity = (numBits + 1);

  Bit parity     = 0;
  Data result    = 0;
  bool isValid   = true;
  bool isMatched = true;

  for (Size i = 0; i < numBitsWithParity; ++i) {
    auto const processBit = [&](Bit const bit) {
      if (i < numBits) {
        result = ((result << 1) | (bit & 1));
        parity ^= (bit & 1);

        if (optionalAddress.has_value() and isMatched) {
          auto const address     = optionalAddress.value();
          auto const bitPosition = (numBits - 1 - i);

          if ((bit & 1) != ((address >> bitPosition) & 1)) {
            isMatched = false;
          }
        }

      } else if (bit != parity) {
        isValid = false;
      }
    };

    auto const optionalBit = readBit(DATA_BIT_NEUTRAL_CYCLES, DATA_BIT_TOTAL_CYCLES, processBit);
    if (not optionalBit.has_value()) {
      return std::nullopt;
    }
  }

  if (sendAck and isMatched) {
    auto const ack    = (isValid ? AckType::ACK : AckType::NAK);
    auto const ackBit = static_cast<Bit>(ack);

    writeDataBit(ackBit);

  } else {
    std::ignore = readDataBit();
  }

  if (not isValid) {
    return std::nullopt;
  }

  return result;
}

template <class T> auto IRAM_ATTR Driver::readBit(Time const neutralCycles, Time const frameCycles, T const& processBit) noexcept -> std::optional<Bit> {
  auto const waitStartCycles = esp_cpu_get_cycle_count();

  while (isBusLow()) {
    auto const currentCycles   = esp_cpu_get_cycle_count();
    auto const differentCycles = (currentCycles - waitStartCycles);
    if (differentCycles > frameCycles) {
      return std::nullopt;
    }
  }

  auto const bitStartCycles = esp_cpu_get_cycle_count();

  while ((esp_cpu_get_cycle_count() - bitStartCycles) < neutralCycles) {}

  Bit const bit = gpio_get_level(static_cast<gpio_num_t>(m_rxPin));

  processBit(bit);

  while ((esp_cpu_get_cycle_count() - bitStartCycles) < frameCycles) {}

  m_lowLevelStartCycles = esp_cpu_get_cycle_count();

  return bit;
}

auto IRAM_ATTR Driver::writeStartBit() noexcept -> bool {
  return writeBit(START_BIT_HIGH_US, START_BIT_TOTAL_US);
}

auto IRAM_ATTR Driver::writeDataBit(Bit const bit) noexcept -> bool {
  auto const pulseWidth = ((bit & 1) ? DATA_BIT_1_HIGH_US : DATA_BIT_0_HIGH_US);

  return writeBit(pulseWidth, DATA_BIT_TOTAL_US);
}

auto IRAM_ATTR Driver::writeField(Data const data, Size const numBits, std::optional<bool> const optionalAck) noexcept -> bool {
  Bit parity = 0;

  for (Size i = 0; i < numBits; ++i) {
    auto const bitPosition = (numBits - 1 - i);
    auto const bit         = static_cast<Bit>((data >> bitPosition) & 1);

    parity ^= (bit & 1);

    writeDataBit(bit);
  }

  writeDataBit(parity);

  if (optionalAck.has_value()) {
    auto const forDevice = optionalAck.value();

    if (forDevice) {
      auto const optionalAckBit = readDataBit();
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

      writeDataBit(ackBit);
    }
  }

  return true;
}

auto IRAM_ATTR Driver::writeBit(Time const pulseCycles, Time const frameCycles) noexcept -> bool {
  auto const waitStartCycles = esp_cpu_get_cycle_count();

  while (isBusHigh()) {
    auto const currentCycles   = esp_cpu_get_cycle_count();
    auto const differentCycles = (currentCycles - waitStartCycles);
    if (differentCycles > frameCycles) {
      return false;
    }
  }

  auto const bitStartCycles = esp_cpu_get_cycle_count();

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 1);
  while ((esp_cpu_get_cycle_count() - bitStartCycles) < pulseCycles) {}

  m_lowLevelStartCycles = esp_cpu_get_cycle_count();

  gpio_set_level(static_cast<gpio_num_t>(m_txPin), 0);
  while ((esp_cpu_get_cycle_count() - bitStartCycles) < frameCycles) {}

  return true;
}

} // namespace iebus
