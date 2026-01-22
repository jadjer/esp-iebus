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

#include <expected>
#include <utility>

#include <iebus/Driver.hpp>
#include <iebus/Message.hpp>
#include <iebus/types.hpp>

/**
 * @namespace iebus
 */
namespace iebus {

/**
 * @class Controller
 * IEBus Controller
 */
class Controller {
private:
  enum class ReadFieldError {
    DATA_READ_ERROR,
    PARITY_BIT_READ_ERROR,
    PARITY_WRONG,
  };

  enum class WriteFieldError {
    ACK_BIT_READ_ERROR,
    ACK_WRONG,
  };

public:
  Controller(Driver& driver, Address address) noexcept;

public:
  /**
   * Read the message from IEBus
   * @return Optional message
   */
  [[nodiscard]] auto readMessage() const noexcept -> std::expected<Message, MessageError>;

private:
  /**
   * Read filed
   * @param bitSize
   * @param sendAck
   * @return
   */
  [[nodiscard]] auto readField(Size bitSize, bool sendAck) const noexcept -> std::expected<Data, ReadFieldError>;

public:
  /**
   * Write a message to IEBus
   * @param message Message
   * @return bool
   */
  [[nodiscard]] auto writeMessage(Message const& message) const noexcept -> std::expected<std::monostate, MessageError>;

private:
  /**
   * Write field
   * @param data
   * @param bitSize
   * @param forDevice
   * @return
   */
  [[nodiscard]] auto writeField(Data data, Size bitSize, bool forDevice) const noexcept -> std::expected<std::monostate, WriteFieldError>;

private:
  Address const m_address;

private:
  Driver& m_driver;
};

} // namespace iebus
