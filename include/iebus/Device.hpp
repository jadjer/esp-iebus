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
// Created by jadjer on 02.02.2026.
//

#pragma once

#include <iebus/types.hpp>
#include <vector>

/**
 * @namespace iebus
 */
namespace iebus {

enum class Command : Byte {
  COMMAND_10 = 0x10,
  COMMAND_11 = 0x11,
  COMMAND_12 = 0x12,
  COMMAND_13 = 0x13,
  COMMAND_20 = 0x20,
  COMMAND_40 = 0x40,
  COMMAND_60 = 0x60,
  COMMAND_70 = 0x70,
  COMMAND_D0 = 0xD0,
};

/**
 * @class Device
 * @brief Physical device abstract class
 */
class Device {
public:
  using MessageList = std::vector<iebus::Message>;

public:
  Device(Address address) noexcept;
  virtual ~Device() noexcept = default;

public:
  /**
   * Get device address
   * @return Address
   */
  [[nodiscard]] auto getAddress() const noexcept -> Address;

public:
  /**
   * Process message from IEBus on physical device
   * @param message Message
   * @return List of message for response tyo IEBus
   */
  [[nodiscard]] virtual auto processMessage(Message const& message) -> MessageList = 0;

protected:
  /**
   * Check message for current physical device
   * @param message Message
   * @return Bool
   */
  [[nodiscard]] auto checkMessageForMe(Message const& message) const -> bool;

protected:
  /**
   * Create new message for response
   * @param target Target device's address
   * @param command Command (data[0])
   * @param length Payload length (length + 1 command bit)
   * @param payload Payload data
   * @return Message
   */
  [[nodiscard]] auto createCommand(Address target, Command command, Size length, Bytes payload) const noexcept -> Message;
  /**
   * Create new broadcast message for response
   * @param command Command (data[0])
   * @param length Payload length (length + 1 command bit)
   * @param payload Payload data
   * @return Message
   */
  [[nodiscard]] auto createBroadcastCommand(Command command, Size length, Bytes payload) const noexcept -> Message;

protected:
  Address const m_address;
};

} // namespace iebus
