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
// Created by jadjer on 08.01.2026.
//

#pragma once

#include <array>
#include <cstdint>

/**
 * @namespace iebus
 */
namespace iebus {

auto constexpr MAX_MESSAGE_SIZE = 256;

using Bit     = std::uint8_t;
using Pin     = std::uint8_t;
using Byte    = std::uint8_t;
using Data    = std::uint16_t;
using Size    = std::uint16_t;
using Time    = std::int64_t;
using Bytes   = std::array<Byte, MAX_MESSAGE_SIZE>;
using Address = std::uint16_t;

enum class AckType : Bit {
  ACK = 0,
  NAK = 1,
};

enum class BitType : Bit {
  BIT_0       = 0,
  BIT_1       = 1,
  BIT_START   = 10,
  BIT_UNKNOWN = 20,
};

enum class BroadcastType : Bit {
  BROADCAST  = 0,
  FOR_DEVICE = 1,
};

} // namespace iebus
