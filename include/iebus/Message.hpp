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

/**
 * @namespace iebus
 */
namespace iebus {

namespace {

auto constexpr MAX_MESSAGE_SIZE = 256;

}

using Bit     = std::uint8_t;
using Pin     = std::uint8_t;
using Byte    = std::uint8_t;
using Data    = std::uint16_t;
using Size    = std::size_t;
using Bytes   = std::array<Byte, MAX_MESSAGE_SIZE>;
using Address = std::uint16_t;

enum class BroadcastType : Bit {
  BROADCAST = 0x0,
  DEVICE    = 0x1,
};

enum class AckType : Bit {
  ACK = 0x0,
  NAK = 0x1,
};

enum class ControlType : Byte {
  READ_SLAVE_STATUS            = 0x0,
  READ_DATA_AND_LOCK           = 0x3,
  READ_LOCK_ADDRESS_LOW_ORDER  = 0x4,
  READ_LOCK_ADDRESS_HIGH_ORDER = 0x5,
  READ_SLAVE_STATUS_AND_UNLOCK = 0x6,
  READ_DATA                    = 0x7,
  WRITE_COMMAND_AND_LOCK       = 0xA,
  WRITE_DATA_AND_LOCK          = 0xB,
  WRITE_COMMAND                = 0xE,
  WRITE_DATA                   = 0xF,
};

/**
 * @class Message
 */
struct Message {
  Address master          = 0;
  Address slave           = 0;
  BroadcastType broadcast = BroadcastType::DEVICE;
  ControlType control     = ControlType::WRITE_DATA;
  Size length             = 0;
  Bytes data              = {};
};

enum class MessageError {
  DRIVER_DISABLED          = 0,
  BUS_IS_BUSY              = 1,
  START_BIT_READ_ERROR     = 10,
  START_BIT_WRITE_ERROR    = 11,
  BROADCAST_BIT_READ_ERROR = 20,
  BROADCAST_BIT_WRITE_ERROR = 21,
  MASTER_FIELD_READ_ERROR  = 30,
  MASTER_FIELD_WRITE_ERROR  = 31,
  SLAVE_FIELD_READ_ERROR   = 40,
  SLAVE_FIELD_WRITE_ERROR   = 41,
  CONTROL_FIELD_READ_ERROR = 50,
  CONTROL_FIELD_WRITE_ERROR = 51,
  LENGTH_FIELD_READ_ERROR  = 60,
  LENGTH_FIELD_WRITE_ERROR  = 61,
  DATA_FIELD_READ_ERROR    = 70,
  DATA_FIELD_WRITE_ERROR    = 71,
};

} // namespace iebus
