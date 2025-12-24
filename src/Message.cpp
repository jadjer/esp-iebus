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
// Created by jadjer on 24.12.2025.
//

#include "iebus/Message.hpp"

#include <format>
#include <string>

namespace iebus {

namespace {

auto formatBroadcastType(BroadcastType type) -> std::string {
  switch (type) {
  case BroadcastType::BROADCAST:
    return "B";
  case BroadcastType::FOR_DEVICE:
    return "D";
  }

  return "U";
}

auto formatBytesHex(Bytes bytes) -> std::string {
  std::string result;

  Size const count = bytes.size();

  for (Size i = 0; i < bytes.size(); ++i) {
    result += std::format("{:02X}", bytes[i]);

    if (i < count - 1) {
      result += " ";
    }
  }

  return result;
}

} // namespace

auto Message::toString() -> std::string {
  return std::format("{} M{:#06x} S{:#06x} C{:#04x} L{} [{}]", formatBroadcastType(broadcast), master, slave, control, dataLength, formatBytesHex(data));
}

} // namespace iebus
