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
// Created by jadjer on 17.01.2026.
//

#include "iebus/Processor.hpp"

namespace iebus {

namespace {

Address constexpr BROADCAST_ADDRESS = 0xFFF;

} // namespace

Processor::Processor(Address const address) noexcept : m_address(address) {
}

auto Processor::processMessage(Message const& message) noexcept -> MessageList {
  if (not checkSlaveAddress(message)) {
    return {};
  }

  auto const command = static_cast<Command>(message.data[0]);

  if (command == Command::START) {
    m_isRegistered = true;
    return handleCommandStart(message);
  }

  if (not m_isRegistered) {
    m_isRegistered = true;
    return createCommandRestart();
  }

  switch (command) {
  case Command::COMMAND_20: return handleCommand20(message);
  case Command::COMMAND_40: return handleCommand40(message);
  case Command::COMMAND_60: return handleCommand60(message);
  default: return {};
  }
}

auto Processor::handleCommandStart(Message const& message) noexcept -> MessageList {
  auto const masterAddress = message.master;
  auto const commandBit = static_cast<Bit>(Command::REGISTER);
  auto const uniqueBit = message.data[1];

  return {
      createCommand(masterAddress, {commandBit, uniqueBit, 0x01, 0x02, 0x85, 0x93}, 6),
  };
}

auto Processor::handleCommand20(Message const& message) noexcept -> MessageList {
  return {};
}

auto Processor::handleCommand40(Message const& message) noexcept -> MessageList {
  return {};
}

auto Processor::handleCommand60(Message const& message) noexcept -> MessageList {
  return {};
}

auto Processor::createCommandRestart() const noexcept -> MessageList {
  auto const commandBit = static_cast<Bit>(Command::RESTART);

  return {
      createBroadcastCommand({commandBit}, 1),
  };
}

auto Processor::checkSlaveAddress(Message const& message) const -> bool {
  auto const slave = message.slave;

  if (slave == BROADCAST_ADDRESS) return true;
  if (slave == m_address) return true;

  return false;
}

auto Processor::createCommand(Address const target, Bytes data, Size length) const noexcept -> Message {
  return Message{.broadcast = BroadcastType::FOR_DEVICE, .master = m_address, .slave = target, .control = ControlType::WRITE_COMMAND, .length = length, .data = data};
}

auto Processor::createBroadcastCommand(Bytes data, Size length) const noexcept -> Message {
  return Message{.broadcast = BroadcastType::BROADCAST, .master = m_address, .slave = BROADCAST_ADDRESS, .control = ControlType::WRITE_COMMAND, .length = length, .data = data};
}

} // namespace iebus
