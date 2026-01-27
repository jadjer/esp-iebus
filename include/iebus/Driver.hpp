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

#include <iebus/Message.hpp>
#include <iebus/Timer.hpp>
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
public:
  using OptBit     = std::optional<Bit>;
  using OptBool    = std::optional<bool>;
  using OptData    = std::optional<Data>;
  using OptTime    = std::optional<Timer::Time>;
  using OptAddress = std::optional<Address>;

public:
  Driver(Pin rx, Pin tx, Pin enable) noexcept;

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
  auto enable() const noexcept -> void;
  /**
   * Disable IEBus transmitter
   */
  auto disable() const noexcept -> void;

public:
  /**
   * START INDICATOR FOR DEBUG
   */
  auto startMessageIndicator() const noexcept -> void;
  /**
   * STOP INDICATOR FOR DEBUG
   */
  auto stopMessageIndicator() const noexcept -> void;

public:
  /**
   * Read start bit from IEBus
   * @return bool
   */
  [[nodiscard]] auto readStartBit() const noexcept -> bool;
  /**
   * Get bit from IEBus
   * @return Bit
   */
  [[nodiscard]] auto readBit() const noexcept -> OptBit;
  /**
   * Get bits data from IEBus
   * @param numBits data size
   * @return Data bits
   */
  [[nodiscard]] auto readField(Size numBits, OptBool optWriteAck = std::nullopt, OptAddress optAddress = std::nullopt) const noexcept -> OptData;

private:
  /**
   * Read pulse width from IEBus
   * @return Pulse width time
   */
  [[nodiscard]] auto readPulseWidth(Timer::Time waitTimeoutUS, Timer::Time pulseTimeoutUS) const noexcept -> OptTime;

public:
  /**
   * Get start bit from IEBus
   * @return
   */
  [[nodiscard]] auto writeStartBit() const noexcept -> bool;
  /**
   * Send bit to IEBus
   * @param bit
   */
  [[nodiscard]] auto writeBit(Bit bit) const noexcept -> bool;
  /**
   * Send data bits to IEBus
   * @param data data bits
   * @param numBits data size
   */
  [[nodiscard]] auto writeField(Data data, Size numBits, OptBool optReadAck = std::nullopt) const noexcept -> bool;

private:
  /**
   * Write pulse width
   * @param pulseWidth
   */
  [[nodiscard]] auto writePulseWidth(Timer::Time waitTimeoutUS, Timer::Time pulseUS, Timer::Time frameUS) const noexcept -> bool;

private:
  Pin const m_rxPin;
  Pin const m_txPin;
  Pin const m_enablePin;

private:
  Timer m_lowTimer;
  Timer m_highTimer;
};

} // namespace iebus
