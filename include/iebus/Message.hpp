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
// Created by jadjer on 29.11.2025.
//

#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace iebus {

auto constexpr MAX_MESSAGE_SIZE = 256;

using Bit = std::uint8_t;
using Byte = std::uint8_t;
using Size = std::size_t;
using Bytes = std::array<Byte, MAX_MESSAGE_SIZE>;
using Address = std::uint16_t;

enum class BroadcastType : Bit {
  BROADCAST = 0,
  FOR_DEVICE = 1,
};

struct Message {
  BroadcastType broadcast;
  Address master;
  Address slave;
  Byte control;
  Size dataLength;
  Bytes data;

  [[maybe_unused]] auto toString() const -> std::string;
};

} // namespace iebus
