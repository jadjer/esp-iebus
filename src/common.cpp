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

namespace iebus {

auto printMessage(Message const& message) -> void {
  auto const broadcast = message.broadcast == iebus::BroadcastType::FOR_DEVICE ? "D" : "B";
  printf("%s %03X %03X %01X %hu [", broadcast, message.master, message.slave, message.control, message.length);
  printf("%02X", message.data[0]);
  for (auto i = 1; i < message.length; ++i) {
    printf(" %02X", message.data[i]);
  }
  printf("]\n");
}

auto messageErrorToString(MessageError const error) -> char const* {
  switch (error) {
  case MessageError::CONTROLLER_DISABLED: return "CONTROLLER_DISABLED";
  case MessageError::START_BIT_READ_ERROR: return "START_BIT_READ_ERROR";
  case MessageError::START_BIT_ARBITRATION_LOST: return "START_BIT_ARBITRATION_LOST";
  case MessageError::START_BIT_IS_FALSE: return "START_BIT_IS_FALSE";
  case MessageError::BROADCAST_BIT_READ_ERROR: return "BROADCAST_BIT_READ_ERROR";
  case MessageError::MASTER_ADDRESS_DATA_READ_ERROR: return "MASTER_ADDRESS_DATA_READ_ERROR";
  case MessageError::MASTER_ADDRESS_PARITY_BIT_READ_ERROR: return "MASTER_ADDRESS_PARITY_BIT_READ_ERROR";
  case MessageError::MASTER_ADDRESS_PARITY_WRONG: return "MASTER_ADDRESS_PARITY_WRONG";
  case MessageError::SLAVE_ADDRESS_DATA_READ_ERROR: return "SLAVE_ADDRESS_DATA_READ_ERROR";
  case MessageError::SLAVE_ADDRESS_PARITY_BIT_READ_ERROR: return "SLAVE_ADDRESS_PARITY_BIT_READ_ERROR";
  case MessageError::SLAVE_ADDRESS_PARITY_WRONG: return "SLAVE_ADDRESS_PARITY_WRONG";
  case MessageError::SLAVE_ADDRESS_ACK_BIT_READ_ERROR: return "SLAVE_ADDRESS_ACK_BIT_READ_ERROR";
  case MessageError::SLAVE_ADDRESS_ACK_WRONG: return "SLAVE_ADDRESS_ACK_WRONG";
  case MessageError::CONTROL_DATA_READ_ERROR: return "CONTROL_DATA_READ_ERROR";
  case MessageError::CONTROL_PARITY_BIT_READ_ERROR: return "CONTROL_PARITY_BIT_READ_ERROR";
  case MessageError::CONTROL_PARITY_WRONG: return "CONTROL_PARITY_WRONG";
  case MessageError::CONTROL_ACK_BIT_READ_ERROR: return "CONTROL_ACK_BIT_READ_ERROR";
  case MessageError::CONTROL_ACK_WRONG: return "CONTROL_ACK_WRONG";
  case MessageError::LENGTH_DATA_READ_ERROR: return "LENGTH_DATA_READ_ERROR";
  case MessageError::LENGTH_PARITY_BIT_READ_ERROR: return "LENGTH_PARITY_BIT_READ_ERROR";
  case MessageError::LENGTH_PARITY_WRONG: return "LENGTH_PARITY_WRONG";
  case MessageError::LENGTH_ACK_BIT_READ_ERROR: return "LENGTH_ACK_BIT_READ_ERROR";
  case MessageError::LENGTH_ACK_WRONG: return "LENGTH_ACK_WRONG";
  case MessageError::DATA_READ_ERROR: return "DATA_READ_ERROR";
  case MessageError::DATA_PARITY_BIT_READ_ERROR: return "DATA_PARITY_BIT_READ_ERROR";
  case MessageError::DATA_PARITY_WRONG: return "DATA_PARITY_WRONG";
  case MessageError::DATA_ACK_BIT_READ_ERROR: return "DATA_ACK_BIT_READ_ERROR";
  case MessageError::DATA_ACK_WRONG: return "DATA_ACK_WRONG";
  default: return "UNKNOWN_ERROR";
  }
}

auto printMessageError(MessageError const messageError) -> void {
  printf("Error occurred: %s\n", messageErrorToString(messageError));
}

} // namespace iebus
