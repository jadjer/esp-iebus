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
// Created by jadjer on 18.01.2026.
//

#include "iebus/common.hpp"

#include <print>

/**
 * @namespace iebus::common
 */
namespace iebus::common {

namespace {

auto controlTypeToString(ControlType const controlType) -> char const* {
  switch (controlType) {
  case ControlType::READ_SLAVE_STATUS: return "RSS";
  case ControlType::READ_DATA_AND_LOCK: return "RD&L";
  case ControlType::READ_LOCK_ADDRESS_LOW_ORDER: return "RLA_L";
  case ControlType::READ_LOCK_ADDRESS_HIGH_ORDER: return "RLA_H";
  case ControlType::READ_SLAVE_STATUS_AND_UNLOCK: return "RSS&U";
  case ControlType::READ_DATA: return "RD";
  case ControlType::WRITE_COMMAND_AND_LOCK: return "WC&L";
  case ControlType::WRITE_DATA_AND_LOCK: return "WD&L";
  case ControlType::WRITE_COMMAND: return "WC";
  case ControlType::WRITE_DATA: return "WD";
  default: return "UNK";
  }
}

auto messageErrorToString(MessageError const messageError) -> char const* {
  switch (messageError) {
  case MessageError::DRIVER_DISABLED: return "DD";
  case MessageError::BUS_IS_BUSY: return "BIB";
  case MessageError::START_BIT_READ_ERROR: return "SBR";
  case MessageError::START_BIT_WRITE_ERROR: return "SBW";
  case MessageError::BROADCAST_BIT_READ_ERROR: return "BBR";
  case MessageError::BROADCAST_BIT_WRITE_ERROR: return "BBW";
  case MessageError::MASTER_FIELD_READ_ERROR: return "MFR";
  case MessageError::MASTER_FIELD_WRITE_ERROR: return "MFW";
  case MessageError::SLAVE_FIELD_READ_ERROR: return "SFR";
  case MessageError::SLAVE_FIELD_WRITE_ERROR: return "SFW";
  case MessageError::CONTROL_FIELD_READ_ERROR: return "CFR";
  case MessageError::CONTROL_FIELD_WRITE_ERROR: return "CFW";
  case MessageError::LENGTH_FIELD_READ_ERROR: return "LFR";
  case MessageError::LENGTH_FIELD_WRITE_ERROR: return "LFW";
  case MessageError::DATA_FIELD_READ_ERROR: return "DFR";
  case MessageError::DATA_FIELD_WRITE_ERROR: return "DFW";
  default: return "UNK";
  }
}

} // namespace

auto printMessage(Message const& message) -> void {
  auto const broadcast = ((message.broadcast == BroadcastType::DEVICE) ? "D" : "B");
  auto const control   = controlTypeToString(message.control);

  std::printf("%s %03X %03X %s [", broadcast, message.master, message.slave, control);
  std::printf("%02X", message.data[0]);
  for (auto i = 1; i < message.length; ++i) {
    std::printf(" %02X", message.data[i]);
  }
  std::printf("]\n");
}

auto printMessageError(const MessageError& messageError) -> void {
  std::printf("E %s\n", messageErrorToString(messageError));
}

} // namespace iebus::common
