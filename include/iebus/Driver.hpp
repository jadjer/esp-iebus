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

#pragma once

#include <cstdint>

#include <iebus/Message.hpp>

namespace iebus {

enum class AcknowledgmentType : Bit {
  ACK = 0,
  NAK = 1,
};

using Data = std::uint16_t;

/**
 * @class Driver
 * IEBus Driver
 */
class Driver {
public:
  using Pin = std::uint8_t;
  using Time = std::int64_t;

public:
  Driver(Pin rx, Pin tx, Pin enable) noexcept;

public:
  /**
   * IEBus transmitter enabled
   * @return bool
   */
  [[nodiscard]] auto isEnabled() const -> bool;
  /**
   * Check if IEBus is high
   * @return bool
   */
  [[nodiscard]] auto isBusHigh() const -> bool;
  /**
   * Check if IEBus is low
   * @return bool
   */
  [[nodiscard]] auto isBusLow() const -> bool;
  /**
   * Check if bus is free
   * @return bool
   */
  [[nodiscard]] auto isBusFree() const -> bool;

public:
  /**
   * Enable IEBus transmitter
   */
  auto enable() -> void;
  /**
   * Disable IEBus transmitter
   */
  auto disable() -> void;

public:
  /**
   * Send start bit to IEBus
   */
  [[nodiscard]] auto receiveStartBit() const -> bool;
  /**
   * Get single bit from IEBus
   * @return Data bit
   */
  [[nodiscard]] auto receiveBit() const -> Bit;
  /**
   * Get bits data from IEBus
   * @param numBits data size
   * @return Data bits
   */
  [[nodiscard]] auto receiveBits(Size numBits) const -> Data;
  /**
   * Wait ack from IEBus
   * @return Ack value
   */
  [[nodiscard]] auto receiveAckBit() const -> AcknowledgmentType;

public:
  /**
   * Get start bit from IEBus
   * @return
   */
  auto transmitStartBit() const -> void;
  /**
   * Send bit to IEBus
   * @param bit single bit data
   */
  auto transmitBit(Bit bit) const -> void;
  /**
   * Send data bits to IEBus
   * @param data data bits
   * @param numBits data size
   */
  auto transmitBits(Data data, Size numBits) const -> void;
  /**
   * Send ack to IEBus
   * @param ack ack value
   */
  auto sendAckBit(AcknowledgmentType ack) const -> void;

private:
  /**
   * Wait before IEBus is change to low level
   */
  auto waitBusLow() const -> void;
  /**
   * Wait before IEBus is change to high level
   */
  auto waitBusHigh() const -> void;

private:
  Pin const m_rxPin;
  Pin const m_txPin;
  Pin const m_enablePin;

private:
  bool m_isEnabled = false;
};

} // namespace iebus
