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
#include <iebus/Device.hpp>
#include <iebus/Driver.hpp>
#include <iebus/types.hpp>
#include <vector>

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
  using Addresses      = std::vector<Address>;
  using NoneOrError    = std::expected<void, MessageError>;
  using MessageOrError = std::expected<Message, MessageError>;

public:
  /**
   * Controller constructor
   * @param driver Driver impl
   * @param address Device address
   */
  Controller(Driver& driver) noexcept;

public:
  /**
   * Register physical device in controller
   * @param device Device
   */
  auto registerDevice(Device const& device) -> void;

public:
  /**
   * Read the message from IEBus
   * @return Message or Message error
   */
  [[nodiscard]] auto readMessage() noexcept -> MessageOrError;
  /**
   * Write the message to IEBus
   * @param message Message
   * @return None or MessageError
   */
  [[nodiscard]] auto writeMessage(Message const& message) noexcept -> NoneOrError;

private:
  Driver& m_driver;

private:
  Addresses m_addresses = {};
};

} // namespace iebus
