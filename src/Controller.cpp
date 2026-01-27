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
// Created by jadjer on 29.11.2025.
//

#include "iebus/Controller.hpp"

#include <esp_attr.h>

namespace iebus {

namespace {

Size constexpr ADDRESS_BITS_SIZE = 12;
Size constexpr CONTROL_BITS_SIZE = 4;
Size constexpr LENGTH_BITS_SIZE  = 8;
Size constexpr DATA_BITS_SIZE    = 8;

} // namespace

Controller::Controller(Driver const& driver, Address const address) noexcept : m_address(address), m_driver(driver) {
}

auto IRAM_ATTR Controller::readMessage() const noexcept -> Controller::OptMessage {
  if (not m_driver.isEnabled()) return std::nullopt;
  if (not m_driver.readStartBit()) return std::nullopt;

  Message message = {};

  m_driver.startMessageIndicator();

  auto const optBroadcastBit = m_driver.readBit();
  if (not optBroadcastBit) {
    m_driver.stopMessageIndicator();
    return std::nullopt;
  }
  message.broadcast = static_cast<BroadcastType>(*optBroadcastBit);

  auto const optMasterData = m_driver.readField(ADDRESS_BITS_SIZE);
  if (not optMasterData) {
    m_driver.stopMessageIndicator();
    return std::nullopt;
  }
  message.master = static_cast<Address>(*optMasterData);

  auto const forDevice = (message.broadcast == BroadcastType::DEVICE);

  auto const optSlaveData = m_driver.readField(ADDRESS_BITS_SIZE, forDevice, m_address);
  if (not optSlaveData.has_value()) {
    m_driver.stopMessageIndicator();
    return std::nullopt;
  }
  message.slave = static_cast<Address>(*optSlaveData);

  auto const isTarget = (forDevice and (message.slave == m_address));

  auto const optControlData = m_driver.readField(CONTROL_BITS_SIZE, isTarget);
  if (not optControlData) {
    m_driver.stopMessageIndicator();
    return std::nullopt;
  }
  message.control = static_cast<ControlType>(*optControlData);

  auto const optLengthData = m_driver.readField(LENGTH_BITS_SIZE, isTarget);
  if (not optLengthData) {
    m_driver.stopMessageIndicator();
    return std::nullopt;
  }
  message.length = static_cast<Size>(*optLengthData);
  message.length = ((message.length == 0) ? 256 : message.length);

  for (Size i = 0; i < message.length; ++i) {
    auto const optData = m_driver.readField(DATA_BITS_SIZE, isTarget);
    if (not optData) {
      m_driver.stopMessageIndicator();
      return std::nullopt;
    }
    message.data[i] = static_cast<Byte>(*optData);
  }

  m_driver.stopMessageIndicator();

  return message;
}

auto IRAM_ATTR Controller::writeMessage(Message const& message) const noexcept -> bool {
  auto const forDevice = (message.broadcast == BroadcastType::DEVICE);

  if (not m_driver.isEnabled()) return false;
  if (not m_driver.isBusFree()) return false;
  if (not m_driver.writeStartBit()) return false;
  if (not m_driver.writeBit(static_cast<Bit>(message.broadcast))) return false;
  if (not m_driver.writeField(static_cast<Data>(message.master), ADDRESS_BITS_SIZE)) return false;
  if (not m_driver.writeField(static_cast<Data>(message.slave), ADDRESS_BITS_SIZE, forDevice)) return false;
  if (not m_driver.writeField(static_cast<Data>(message.control), CONTROL_BITS_SIZE, forDevice)) return false;
  if (not m_driver.writeField(static_cast<Data>(message.length), LENGTH_BITS_SIZE, forDevice)) return false;

  for (Size i = 0; i < message.length; ++i) {
    if (not m_driver.writeField(static_cast<Data>(message.data[i]), DATA_BITS_SIZE, forDevice)) return false;
  }

  return true;
}

} // namespace iebus
