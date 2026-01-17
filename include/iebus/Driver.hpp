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

#include <optional>

#include <iebus/Timer.hpp>
#include <iebus/types.hpp>

/**
 * @namespace iebus
 */
namespace iebus {

/**
 * @class Driver
 * IEBus Driver
 */
class Driver {
public:
  Driver(Pin rx, Pin tx, Pin enable) noexcept;

public:
  /**
   * IEBus transmitter enabled
   * @return bool
   */
  [[nodiscard]] auto isEnabled() const noexcept -> bool;
  /**
   * Check if IEBus is high
   * @return bool
   */
  [[nodiscard]] auto isBusHigh() const noexcept -> bool;
  /**
   * Check if IEBus is low
   * @return bool
   */
  [[nodiscard]] auto isBusLow() const noexcept -> bool;
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
  [[nodiscard]] auto readStartBit() const noexcept -> bool;
  /**
   * Get bits data from IEBus
   * @param numBits data size
   * @return Data bits
   */
  [[nodiscard]] auto readBits(Size numBits) const noexcept -> Data;

public:
  /**
   * Get start bit from IEBus
   * @return
   */
  auto writeStartBit() const noexcept -> void;
  /**
   * Send data bits to IEBus
   * @param data data bits
   * @param numBits data size
   */
  auto writeBits(Data data, Size numBits) const noexcept -> void;

public:
  /**
   * Capture pulse width
   * @return Pulse width
   */
  [[nodiscard]] auto capturePulseWidth() const noexcept -> Time;

private:
  Pin const m_rxPin;
  Pin const m_txPin;
  Pin const m_enablePin;

private:
  bool m_enable;
  Timer m_timer;
};

} // namespace iebus
