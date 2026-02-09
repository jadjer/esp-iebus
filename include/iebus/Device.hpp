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
  [[nodiscard]] virtual auto update() -> MessageList = 0;

protected:
  /**
   * Check message for current physical device
   * @param message Message
   * @return Bool
   */
  [[nodiscard]] auto checkMessageForMe(Message const& message) const -> bool;

protected:
  Address const m_address;
};

} // namespace iebus
