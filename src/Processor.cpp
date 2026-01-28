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

#include <map>
#include <ranges>
#include <span>

namespace iebus {

namespace {

Address constexpr BROADCAST_ADDRESS = 0xFFF;

std::map<Byte, Bytes> configuration = {
    {0x00, {0x0}},
    {0x01, {0x0}},
    {0x02, {0x0}},
};

} // namespace

Processor::Processor(Address const address) noexcept : m_address(address) {
}

auto Processor::processMessage(Message const& message) noexcept -> MessageList {
  if (not checkMessageForMe(message)) return {};

  auto const command = static_cast<Command>(message.data[0]);
  if (command == Command::START) {
    m_isRegistered = true;
    return handleCommandStart(message);
  }

  if (not m_isRegistered) return createCommandRestart();

  switch (command) {
  case Command::COMMAND_20: return handleCommand20(message);
  case Command::COMMAND_40: return handleCommand40(message);
  case Command::COMMAND_60: return handleCommand60(message);
  default: return {};
  }
}

auto Processor::handleCommandStart(Message const& message) noexcept -> MessageList {
  auto const masterAddress = message.master;
  auto const commandBit    = static_cast<Bit>(Command::REGISTER);
  auto const uniqueBit     = message.data[1];

  return {
      createCommand(masterAddress, {commandBit, uniqueBit, 0x01, 0x02, 0x85, 0x93}, 6),
      createCommand(masterAddress, {0x40, 0xC0, 0x20, 0x02}, 4),
      createCommand(masterAddress, {0x40, 0x02, 0x10}, 3),
      createCommand(masterAddress, {0x40, 0x06, 0x10}, 3),
      createCommand(masterAddress, {0x40, 0xC0, 0x10}, 3),
      createCommand(masterAddress, {0x40, 0x83, 0x10}, 3),
      createCommand(masterAddress, {0x40, 0x02, 0x00}, 3),
      createCommand(masterAddress, {0x40, 0x06, 0x00}, 3),
      createCommand(masterAddress, {0x40, 0xC0, 0x00}, 3),
      createCommand(masterAddress, {0x40, 0x83, 0x00}, 3),
      createCommand(masterAddress, {0x40, 0x02, 0x02, 0x00}, 4),
      createCommand(masterAddress, {0x40, 0x06, 0x02, 0x00, 0x01}, 5),
      createCommand(masterAddress, {0x40, 0x06, 0x02, 0x00, 0x02}, 5),
      createCommand(masterAddress, {0x40, 0x06, 0x02, 0x00, 0x10}, 5),
      createCommand(masterAddress, {0x13, 0xFF}, 2),
      createCommand(masterAddress, {0xD0, 0x31, 0x0B, 0x00}, 4),
  };
}

auto Processor::handleCommand20(Message const& message) noexcept -> MessageList {
  if (message.length < 2) return {};

  auto const masterAddress = message.master;
  auto const command       = message.data[0];
  auto const index         = message.data[1];
  auto const payloadLength = (message.length - 2);
  auto const payload       = std::span(message.data).subspan(2, payloadLength);

  std::copy_n(payload.begin(), payloadLength, configuration[index].begin());

  if (index == 0) {}
  if (index == 1) {}
  if (index == 2) {}

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

auto Processor::checkMessageForMe(Message const& message) const -> bool {
  auto const address = message.slave;
  if (address == BROADCAST_ADDRESS) return true;
  if (address == m_address) return true;
  return false;
}

auto Processor::createCommand(Address const target, Bytes const data, Size const length) const noexcept -> Message {
  return {
      .master    = m_address,
      .slave     = target,
      .broadcast = BroadcastType::DEVICE,
      .control   = ControlType::WRITE_COMMAND,
      .length    = length,
      .data      = data,
  };
}

auto Processor::createBroadcastCommand(Bytes const data, Size const length) const noexcept -> Message {
  return {
      .master    = m_address,
      .slave     = BROADCAST_ADDRESS,
      .broadcast = BroadcastType::BROADCAST,
      .control   = ControlType::WRITE_COMMAND,
      .length    = length,
      .data      = data,
  };
}

} // namespace iebus
