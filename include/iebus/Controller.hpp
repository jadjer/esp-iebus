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
public:
  Controller(Driver const& driver, Address address) noexcept;

public:
  /**
   * Read the message from IEBus
   * @return Optional message
   */
  [[nodiscard]] auto readMessage() const noexcept -> std::optional<Message>;
  /**
   * Write a message to IEBus
   * @param message Message
   * @return bool
   */
  [[nodiscard]] [[maybe_unused]] auto writeMessage(Message const& message) const noexcept -> bool;

private:
  /**
   * Read filed and verify
   * @param bitSize
   * @param sendAck
   * @return
   */
  [[nodiscard]] auto readVerifiedField(Size bitSize, bool sendAck) const noexcept -> std::optional<Data>;

private:
  Address const m_address;

private:
  Driver const& m_driver;
};

} // namespace iebus
