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

auto constexpr ADDRESS_BITS_SIZE = 12;
auto constexpr CONTROL_BITS_SIZE = 4;
auto constexpr LENGTH_BITS_SIZE  = 8;
auto constexpr DATA_BITS_SIZE    = 8;

} // namespace

Controller::Controller(Driver const& driver, Address const address) noexcept : m_address(address), m_driver(driver) {
}

auto IRAM_ATTR Controller::readMessage() const noexcept -> std::expected<Message, MessageError> {
  if (not m_driver.isEnabled()) {
    return std::unexpected(MessageError::CONTROLLER_DISABLED);
  }

  // START_BIT
  auto const isStarted = m_driver.readStartBit();
  if (not isStarted) {
    return std::unexpected(MessageError::START_BIT_READ_ERROR);
  }

  m_driver.enableTest();

  Message message = {};

  // BROADCAST
  auto const optionalBroadcastBit = m_driver.readBit();
  if (not optionalBroadcastBit.has_value()) {
    m_driver.disableTest();
    return std::unexpected(MessageError::BROADCAST_BIT_READ_ERROR);
  }

  auto const broadcastBit = optionalBroadcastBit.value();
  message.broadcast       = static_cast<BroadcastType>(broadcastBit);

  auto const forDevice = static_cast<bool>(broadcastBit);

  // MASTER
  auto const optionalMasterData = m_driver.readField(ADDRESS_BITS_SIZE, false);
  if (not optionalMasterData.has_value()) {
    m_driver.disableTest();
    return std::unexpected(MessageError::MASTER_ADDRESS_FIELD_READ_ERROR);
  }

  auto const masterData = optionalMasterData.value();
  message.master        = static_cast<Address>(masterData);

  // SLAVE
  auto const optionalSlaveData = m_driver.readField(ADDRESS_BITS_SIZE, forDevice, m_address);
  if (not optionalSlaveData.has_value()) {
    m_driver.disableTest();
    return std::unexpected(MessageError::SLAVE_ADDRESS_FIELD_READ_ERROR);
  }

  auto const slaveData = optionalSlaveData.value();
  message.slave        = static_cast<Address>(slaveData);

  auto const isTarget = (forDevice and (message.slave == m_address));

  // CONTROL
  auto const optionalControlData = m_driver.readField(CONTROL_BITS_SIZE, isTarget);
  if (not optionalControlData.has_value()) {
    m_driver.disableTest();
    return std::unexpected(MessageError::CONTROL_FIELD_READ_ERROR);
  }

  auto const controlData = optionalControlData.value();
  message.control        = static_cast<ControlType>(controlData);

  // LENGTH
  auto const optionalLengthData = m_driver.readField(LENGTH_BITS_SIZE, isTarget);
  if (not optionalLengthData.has_value()) {
    m_driver.disableTest();
    return std::unexpected(MessageError::LENGTH_FIELD_READ_ERROR);
  }

  auto const lengthData = optionalLengthData.value();
  message.length        = ((lengthData == 0) ? 256 : static_cast<Size>(lengthData));

  // DATA
  for (Size i = 0; i < message.length; ++i) {
    auto const optionalData = m_driver.readField(DATA_BITS_SIZE, isTarget);
    if (not optionalData.has_value()) {
      m_driver.disableTest();
      return std::unexpected(MessageError::DATA_FIELD_READ_ERROR);
    }

    auto const data = optionalData.value();
    message.data[i] = static_cast<Byte>(data);
  }

  m_driver.disableTest();

  return message;
}

auto IRAM_ATTR Controller::writeMessage(Message const& message) const noexcept -> std::expected<std::monostate, MessageError> {
  if (not m_driver.isEnabled()) {
    return std::unexpected(MessageError::CONTROLLER_DISABLED);
  }

  if (not m_driver.isBusFree()) {
    return std::unexpected(MessageError::BUS_IS_BUSY);
  }

  if (auto const isWritten = m_driver.writeStartBit(); not isWritten) {
    return std::unexpected(MessageError::START_BIT_WRITE_ERROR);
  }

  if (auto const isWritten = m_driver.writeBit(static_cast<Bit>(message.broadcast)); not isWritten) {
    return std::unexpected(MessageError::BROADCAST_BIT_WRITE_ERROR);
  }

  if (auto const isWritten = m_driver.writeField(static_cast<Data>(message.master), ADDRESS_BITS_SIZE, false); not isWritten) {
    return std::unexpected(MessageError::MASTER_ADDRESS_FIELD_WRITE_ERROR);
  }

  auto const isTargeted = (message.broadcast == BroadcastType::FOR_DEVICE);

  if (auto const isWritten = m_driver.writeField(static_cast<Data>(message.slave), ADDRESS_BITS_SIZE, isTargeted); not isWritten) {
    return std::unexpected(MessageError::SLAVE_ADDRESS_FIELD_WRITE_ERROR);
  }

  if (auto const isWritten = m_driver.writeField(static_cast<Data>(message.control), CONTROL_BITS_SIZE, isTargeted); not isWritten) {
    return std::unexpected(MessageError::CONTROL_FIELD_WRITE_ERROR);
  }

  if (auto const isWritten = m_driver.writeField(static_cast<Data>(message.length), LENGTH_BITS_SIZE, isTargeted); not isWritten) {
    return std::unexpected(MessageError::LENGTH_FIELD_WRITE_ERROR);
  }

  for (Size i = 0; i < message.length; ++i) {
    if (auto const isWritten = m_driver.writeField(static_cast<Data>(message.data[i]), DATA_BITS_SIZE, isTargeted); not isWritten) {
      return std::unexpected(MessageError::DATA_FIELD_WRITE_ERROR);
    }
  }

  return std::monostate();
}

} // namespace iebus
