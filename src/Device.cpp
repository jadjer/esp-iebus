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
// Created by jadjer on 02.02.2026.
//

#include "iebus/Device.hpp"

namespace iebus {

Device::Device(Address const address) noexcept : m_address(address) {
}

auto Device::getAddress() const noexcept -> Address {
  return m_address;
}

auto Device::checkMessageForMe(Message const& message) const -> bool {
  if (message.broadcast == BroadcastType::BROADCAST) return true;
  if (message.slave == m_address) return true;

  return false;
}

} // namespace iebus
