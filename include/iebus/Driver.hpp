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

#pragma once

#include <driver/gpio_filter.h>
#include <iebus/types.hpp>
#include <optional>

/**
 * @namespace iebus
 */
namespace iebus {

/**
 * @class Driver
 * IEBus Driver
 */
class Driver {
private:
  using Filter = gpio_glitch_filter_handle_t;

public:
  Driver(Pin rx, Pin tx, Pin enable) noexcept;
  ~Driver() noexcept;

public:
  /**
   * IEBus transmitter enabled
   * @return bool
   */
  [[nodiscard]] auto isEnabled() const noexcept -> bool;
  /**
   * Check if IEBus is low
   * @return bool
   */
  [[nodiscard]] auto isBusLow() const noexcept -> bool;
  /**
   * Check if IEBus is high
   * @return bool
   */
  [[nodiscard]] auto isBusHigh() const noexcept -> bool;

public:
  /**
   * Check if bus is free
   * @return bool
   */
  [[nodiscard]] auto isBusFree() const noexcept -> bool;

public:
  /**
   * Enable IEBus transmitter
   */
  auto enable() noexcept -> void;
  /**
   * Disable IEBus transmitter
   */
  auto disable() noexcept -> void;

public:
  /**
   * Read start bit from IEBus
   * @return bool
   */
  [[nodiscard]] auto readStartBit() noexcept -> bool;
  /**
   * Get bit from IEBus
   * @return Bit
   */
  [[nodiscard]] auto readDataBit() noexcept -> std::optional<Bit>;
  /**
   * Get bits data from IEBus
   * @param numBits data size
   * @return Data bits
   */
  [[nodiscard]] auto readField(Size numBits, bool sendAck, std::optional<Address> optionalAddress = std::nullopt) noexcept -> std::optional<Data>;

private:
  /**
   * Read bit from IEBus
   * @return Bit
   */
  template <class T> [[nodiscard]] auto readBit(Time neutralCycles, Time frameCycles, T const& processBit) noexcept -> std::optional<Bit>;

public:
  /**
   * Get start bit from IEBus
   * @return
   */
  auto writeStartBit() noexcept -> bool;
  /**
   * Send bit to IEBus
   * @param bit
   */
  auto writeDataBit(Bit bit) noexcept -> bool;
  /**
   * Send data bits to IEBus
   * @param data data bits
   * @param numBits data size
   */
  auto writeField(Data data, Size numBits, std::optional<bool> optionalAck = std::nullopt) noexcept -> bool;

private:
  /**
   * Write pulse width
   * @param pulseWidth
   */
  auto writeBit(Time pulseCycles, Time frameCycles) noexcept -> bool;

private:
  Pin const m_rxPin;
  Pin const m_txPin;
  Pin const m_enablePin;

private:
  bool m_enable   = false;
  Filter m_filter = nullptr;
  Time m_lowLevelStartCycles = 0;
};

} // namespace iebus
