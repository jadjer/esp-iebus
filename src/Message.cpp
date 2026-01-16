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
// Created by jadjer on 16.01.2026.
//

#include "iebus/Message.hpp"

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

} // namespace iebus
