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

Controller::Controller(Driver& driver, Address const address) noexcept : m_address(address), m_driver(driver) {
}

auto IRAM_ATTR Controller::readMessage() noexcept -> Controller::MessageOrError {
  Message message = {};

  if (not m_driver.isEnabled()) return std::unexpected(MessageError::DRIVER_DISABLED);
  if (not m_driver.readStartBit()) return std::unexpected(MessageError::START_BIT_READ_ERROR);

  auto const optBroadcastBit = m_driver.readBit();
  if (not optBroadcastBit) return std::unexpected(MessageError::BROADCAST_BIT_READ_ERROR);
  message.broadcast = static_cast<BroadcastType>(*optBroadcastBit);

  auto const optMasterData = m_driver.readField(ADDRESS_BITS_SIZE);
  if (not optMasterData) return std::unexpected(MessageError::MASTER_FIELD_READ_ERROR);
  message.master = static_cast<Address>(*optMasterData);

  auto const forDevice = (message.broadcast == BroadcastType::DEVICE);

  auto const optSlaveData = m_driver.readField(ADDRESS_BITS_SIZE, forDevice, m_address);
  if (not optSlaveData.has_value()) return std::unexpected(MessageError::SLAVE_FIELD_READ_ERROR);
  message.slave = static_cast<Address>(*optSlaveData);

  auto const isTarget = (forDevice and (message.slave == m_address));

  auto const optControlData = m_driver.readField(CONTROL_BITS_SIZE, isTarget);
  if (not optControlData) return std::unexpected(MessageError::CONTROL_FIELD_READ_ERROR);
  message.control = static_cast<ControlType>(*optControlData);

  auto const optLengthData = m_driver.readField(LENGTH_BITS_SIZE, isTarget);
  if (not optLengthData) return std::unexpected(MessageError::LENGTH_FIELD_READ_ERROR);
  message.length = static_cast<Size>(*optLengthData);
  message.length = ((message.length == 0) ? 256 : message.length);

  for (Size i = 0; i < message.length; ++i) {
    auto const optData = m_driver.readField(DATA_BITS_SIZE, isTarget);
    if (not optData) return std::unexpected(MessageError::DATA_FIELD_READ_ERROR);
    message.data[i] = static_cast<Byte>(*optData);
  }

  return message;
}

auto IRAM_ATTR Controller::writeMessage(Message const& message) noexcept -> Controller::NoneOrError {
  auto const forDevice = (message.broadcast == BroadcastType::DEVICE);

  if (not m_driver.isEnabled()) return std::unexpected(MessageError::DRIVER_DISABLED);
  if (not m_driver.isBusFree()) return std::unexpected(MessageError::BUS_IS_BUSY);

  if (not m_driver.writeStartBit()) return std::unexpected(MessageError::START_BIT_WRITE_ERROR);
  if (not m_driver.writeBit(static_cast<Bit>(message.broadcast))) return std::unexpected(MessageError::BROADCAST_BIT_WRITE_ERROR);
  if (not m_driver.writeField(static_cast<Data>(message.master), ADDRESS_BITS_SIZE)) return std::unexpected(MessageError::MASTER_FIELD_WRITE_ERROR);
  if (not m_driver.writeField(static_cast<Data>(message.slave), ADDRESS_BITS_SIZE, forDevice)) return std::unexpected(MessageError::SLAVE_FIELD_WRITE_ERROR);
  if (not m_driver.writeField(static_cast<Data>(message.control), CONTROL_BITS_SIZE, forDevice)) return std::unexpected(MessageError::CONTROL_FIELD_WRITE_ERROR);
  if (not m_driver.writeField(static_cast<Data>(message.length), LENGTH_BITS_SIZE, forDevice)) return std::unexpected(MessageError::LENGTH_FIELD_WRITE_ERROR);

  for (Size i = 0; i < message.length; ++i) {
    if (not m_driver.writeField(static_cast<Data>(message.data[i]), DATA_BITS_SIZE, forDevice)) return std::unexpected(MessageError::DATA_FIELD_WRITE_ERROR);
  }

  return {};
}

} // namespace iebus
