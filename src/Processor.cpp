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

Processor::Processor(Address const address) noexcept : m_address(address), m_isRegistered(false) {
}

auto Processor::processMessage(Message const& message) noexcept -> std::optional<Message> {
  auto const command = message.data[0];

  if (command == 10) {
    m_isRegistered = true;
    handleCommand10(message);
  }

  if (not m_isRegistered) {
    return Message{BroadcastType::BROADCAST, m_address, 0xFFF, static_cast<Byte>(ControlType::WRITE_COMMAND), 1, {0x12}};
  }

  switch (command) {
  case 0x20:
    return handleCommand20(message);
  case 0x40:
    return handleCommand40(message);
  case 0x60:
    return handleCommand60(message);
  case 0x70:
    return handleCommand70(message);
  default:
    return std::nullopt;
  }
}

auto Processor::handleCommand10(Message const& message) noexcept -> std::optional<Message> {
  auto const uniqueValue = message.data[1];

  return createResponse(message.master, 6, {0x11, uniqueValue, 0x01, 0x02, 0x85, 0x93});
}

auto Processor::handleCommand20(Message const& message) noexcept -> std::optional<Message> {
  auto const command = message.data[0];
  auto const index = message.data[1];

  if (command == 0x20 and index == 0x02) {
    return createResponse(message.master, 4, {0x40, 0xC0, 0x20, 0x02});
  }

  return std::nullopt;
}

auto Processor::handleCommand40(Message const& message) noexcept -> std::optional<Message> {
  return std::nullopt;
}

auto Processor::handleCommand60(Message const& message) noexcept -> std::optional<Message> {
  return std::nullopt;
}

auto Processor::handleCommand70(Message const& message) noexcept -> std::optional<Message> {
  return std::nullopt;
}

auto Processor::createResponse(Address const target, Size const length, Bytes const payload) const noexcept -> Message {
  return Message{
      BroadcastType::FOR_DEVICE, m_address, target, static_cast<Byte>(ControlType::WRITE_COMMAND), length, payload,
  };
}

} // namespace iebus
