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
#include <optional>

namespace iebus {

class Processor {
public:
  Processor(Address address) noexcept;

public:
  auto processMessage(Message const& message) noexcept -> std::optional<Message>;

private:
  auto handleCommand10(Message const& message) noexcept -> std::optional<Message>;
  auto handleCommand20(Message const& message) noexcept -> std::optional<Message>;
  auto handleCommand40(Message const& message) noexcept -> std::optional<Message>;
  auto handleCommand60(Message const& message) noexcept -> std::optional<Message>;
  auto handleCommand70(Message const& message) noexcept -> std::optional<Message>;

private:
  [[nodiscard]] auto createResponse(Address target, Size length, Bytes payload) const noexcept -> Message;

private:
  Address const m_address;

private:
  bool m_isRegistered;
};

} // namespace iebus
