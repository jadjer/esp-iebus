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

#include "iebus/Device.hpp"

namespace iebus {

namespace {

Address constexpr BROADCAST_ADDRESS = 0xFFF;

}

Device::Device(Address const address) noexcept : m_address(address) {
}

auto Device::getAddress() const noexcept -> Address {
  return m_address;
}

auto Device::checkMessageForMe(Message const& message) const -> bool {
  auto const isBroadcast   = (message.broadcast == BroadcastType::BROADCAST);
  auto const targetAddress = message.slave;

  if (isBroadcast) return true;
  if (targetAddress == m_address) return true;

  return false;
}

auto Device::createCommand(Address const target, Command const command, Size const length, Bytes const payload) const noexcept -> Message {
  Message message = {
      .master    = m_address,
      .slave     = target,
      .broadcast = BroadcastType::DEVICE,
      .control   = ControlType::WRITE_COMMAND,
      .length    = (length + 1),
      .data      = {},
  };

  message.data[0] = static_cast<Bit>(command);

  for (Size i = 0; ((i < length) and (i < (MAX_MESSAGE_SIZE - 1))); ++i) {
    message.data[i + 1] = payload[i];
  }

  return message;
}

auto Device::createBroadcastCommand(Command const command, Size const length, Bytes const payload) const noexcept -> Message {
  auto message = createCommand(BROADCAST_ADDRESS, command, length, payload);

  message.broadcast = BroadcastType::BROADCAST;

  return message;
}

} // namespace iebus
