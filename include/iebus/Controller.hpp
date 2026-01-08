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

#include <expected>

#include <iebus/Driver.hpp>
#include <iebus/Message.hpp>

namespace iebus {

enum class MessageReadError : Bit {
  CONTROLLER_DISABLED = 0,
  START_BIT_READ_ERROR = 10,
  BROADCAST_BIT_READ_ERROR = 20,
  MASTER_ADDRESS_DATA_READ_ERROR = 30,
  MASTER_ADDRESS_PARITY_BIT_READ_ERROR = 31,
  MASTER_ADDRESS_PARITY_WRONG = 32,
  SLAVE_ADDRESS_DATA_READ_ERROR = 40,
  SLAVE_ADDRESS_PARITY_BIT_READ_ERROR = 41,
  SLAVE_ADDRESS_ACK_BIT_READ_ERROR = 42,
  SLAVE_ADDRESS_PARITY_WRONG = 43,
  CONTROL_DATA_READ_ERROR = 50,
  CONTROL_PARITY_BIT_READ_ERROR = 51,
  CONTROL_ACK_BIT_READ_ERROR = 52,
  CONTROL_PARITY_WRONG = 53,
  LENGTH_DATA_READ_ERROR = 60,
  LENGTH_PARITY_BIT_READ_ERROR = 61,
  LENGTH_ACK_BIT_READ_ERROR = 62,
  LENGTH_PARITY_WRONG = 63,
  DATA_READ_ERROR = 70,
  DATA_PARITY_BIT_READ_ERROR = 71,
  DATA_ACK_BIT_READ_ERROR = 72,
  DATA_PARITY_WRONG = 73,
};

/**
 * @class Controller
 * IEBus Controller
 */
class Controller {

public:
  Controller(Driver::Pin rx, Driver::Pin tx, Driver::Pin enable, Address address) noexcept;

public:
  /**
   * Enable IEBus driver
   */
  auto enable() -> void;
  /**
   * Enable IEBus driver
   */
  auto disable() -> void;

public:
  /**
   * Check if IEBus controller enabled
   * @return bool
   */
  [[nodiscard]] auto isEnabled() const -> bool;

public:
  /**
   * Read the message from IEBus
   * @return Optional message
   */
  [[nodiscard]] auto readMessage() -> std::expected<Message, MessageReadError>;
  /**
   * Write a message to IEBus
   * @param message Message
   * @return bool
   */
  [[nodiscard]] auto writeMessage(Message const& message) -> bool;

private:
  Address const m_address;

private:
  Driver m_driver;
};

} // namespace iebus
