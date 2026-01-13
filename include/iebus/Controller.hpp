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
  virtual ~Controller() noexcept = default;

public:
  [[nodiscard]] auto registerOnMaster() const -> bool;

public:
  /**
   * Read the message from IEBus
   * @return Optional message
   */
  [[nodiscard]] auto readMessage() const noexcept -> std::expected<Message, BitError>;
  /**
   * Write a message to IEBus
   * @param message Message
   * @return bool
   */
  [[nodiscard]] [[maybe_unused]] auto writeMessage(Message const& message) const noexcept -> std::expected<bool, ErrorType>;

private:
  Driver const& m_driver;
  Address const m_address;
};

} // namespace iebus
