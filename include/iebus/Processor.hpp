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

#pragma once

#include <iebus/Message.hpp>
#include <iebus/types.hpp>
#include <vector>

namespace iebus {

enum class Command : Bit {
  START      = 0x10,
  REGISTER   = 0x11,
  RESTART    = 0x12,
  COMMAND_20 = 0x20,
  COMMAND_40 = 0x40,
  COMMAND_60 = 0x60,
};

class Processor {
public:
  using MessageList = std::vector<Message>;

public:
  Processor(Address address) noexcept;

public:
  [[nodiscard]] auto processMessage(Message const& message) noexcept -> MessageList;

private:
  [[nodiscard]] auto handleCommandStart(Message const& message) noexcept -> MessageList;
  [[nodiscard]] auto handleCommand20(Message const& message) noexcept -> MessageList;
  [[nodiscard]] auto handleCommand40(Message const& message) noexcept -> MessageList;
  [[nodiscard]] auto handleCommand60(Message const& message) noexcept -> MessageList;

private:
  [[nodiscard]] auto createCommandRestart() const noexcept -> MessageList;

private:
  [[nodiscard]] auto checkSlaveAddress(Message const& message) const -> bool;

private:
  [[nodiscard]] auto createCommand(Address target, Bytes data, Size length) const noexcept -> Message;
  [[nodiscard]] auto createBroadcastCommand(Bytes data, Size length) const noexcept -> Message;

private:
  Address const m_address;

private:
  bool m_isRegistered = false;
  bool m_isRegisterRequest = false;
};

} // namespace iebus
