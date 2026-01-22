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

#include <bit>
#include <ranges>

#include "iebus/Message.hpp"

namespace iebus {

namespace {

auto constexpr TAG = "IEBusController";

auto constexpr ADDRESS_BITS_SIZE = 12;
auto constexpr CONTROL_BITS_SIZE = 4;
auto constexpr LENGTH_BITS_SIZE  = 8;
auto constexpr DATA_BITS_SIZE    = 8;

auto calculateParity(Data const data) -> Bit {
  return std::popcount(data) & 1;
}

auto checkParity(Data const data, Bit const parity) -> bool {
  auto const calculatedParity = calculateParity(data);
  auto const isValid          = (calculatedParity == parity);

  return isValid;
}

} // namespace

Controller::Controller(Driver& driver, Address const address) noexcept : m_address(address), m_driver(driver) {
}

auto Controller::readMessage() const noexcept -> std::expected<Message, MessageError> {
  if (not m_driver.isEnabled()) {
    return std::unexpected(MessageError::CONTROLLER_DISABLED);
  }

  // START_BIT
  auto const isStarted = m_driver.readStartBit();
  if (not isStarted) {
    return std::unexpected(MessageError::START_BIT_IS_FALSE);
  }

  Message message = {};

//  Message message = {
//      BroadcastType::BROADCAST, 0x130, 0xFFF, ControlType::WRITE_COMMAND, 3,{0x10, 0x1A, 0x01}
//  };

//  Message message = {
//      BroadcastType::BROADCAST, 0x130, 0xFFF, ControlType::WRITE_COMMAND, 12, {0x60, 0xC0, 0x00, 0x4F, 0xFF, 0x00, 0x00, 0x22, 0x00, 0x22, 0x00, 0x00}
//  };

//  return message;

  // BROADCAST
  auto const optionalBroadcastBit = m_driver.readBits(1);
  if (not optionalBroadcastBit.has_value()) {
    return std::unexpected(MessageError::BROADCAST_BIT_READ_ERROR);
  }
  auto const broadcastBit = optionalBroadcastBit.value();

  message.broadcast = static_cast<BroadcastType>(broadcastBit);

  // MASTER
  auto const optionalMasterData = m_driver.readBits(ADDRESS_BITS_SIZE);
  if (not optionalMasterData.has_value()) {
    return std::unexpected(MessageError::MASTER_ADDRESS_DATA_READ_ERROR);
  }
  auto const masterData = optionalMasterData.value();

  auto const optionalMasterParityBit = m_driver.readBits(1);
  if (not optionalMasterParityBit.has_value()) {
    return std::unexpected(MessageError::MASTER_ADDRESS_PARITY_BIT_READ_ERROR);
  }
  auto const masterParityBit = optionalMasterParityBit.value();

  auto const isValidMasterParity = checkParity(masterData, masterParityBit);
  if (not isValidMasterParity) {
    return std::unexpected(MessageError::MASTER_ADDRESS_PARITY_WRONG);
  }
  message.master = static_cast<Address>(masterData);

  // SLAVE
  auto const optionalSlaveData = m_driver.readBits(ADDRESS_BITS_SIZE);
  if (not optionalSlaveData.has_value()) {
    return std::unexpected(MessageError::SLAVE_ADDRESS_DATA_READ_ERROR);
  }
  auto const slaveData = optionalSlaveData.value();

  auto const optionalSlaveParityBit = m_driver.readBits(1);
  if (not optionalSlaveParityBit.has_value()) {
    return std::unexpected(MessageError::SLAVE_ADDRESS_PARITY_BIT_READ_ERROR);
  }
  auto const slaveParityBit = optionalSlaveParityBit.value();

  auto const isValidSlaveParity = checkParity(slaveData, slaveParityBit);

  message.slave = static_cast<Address>(slaveData);

  auto const isTargeted = (message.broadcast == BroadcastType::FOR_DEVICE and message.slave == m_address);

  if (isTargeted) {
    m_driver.writeBits(static_cast<Data>(isValidSlaveParity ? AckType::ACK : AckType::NAK), 1);
  } else {
    std::ignore = m_driver.readBits(1);
  }

  if (not isValidSlaveParity) {
    return std::unexpected(MessageError::SLAVE_ADDRESS_PARITY_WRONG);
  }

  // CONTROL
  auto const expectedControlData = readField(CONTROL_BITS_SIZE, isTargeted);
  if (not expectedControlData.has_value()) {
    auto const error = expectedControlData.error();
    switch (error) {
    case ReadFieldError::DATA_READ_ERROR: return std::unexpected(MessageError::CONTROL_DATA_READ_ERROR);
    case ReadFieldError::PARITY_BIT_READ_ERROR: return std::unexpected(MessageError::CONTROL_PARITY_BIT_READ_ERROR);
    case ReadFieldError::PARITY_WRONG: return std::unexpected(MessageError::CONTROL_PARITY_WRONG);
    }
  }
  auto const controlData = expectedControlData.value();

  message.control = static_cast<ControlType>(controlData);

  // LENGTH
  auto const expectedLengthData = readField(LENGTH_BITS_SIZE, isTargeted);
  if (not expectedLengthData.has_value()) {
    auto const error = expectedControlData.error();
    switch (error) {
    case ReadFieldError::DATA_READ_ERROR: return std::unexpected(MessageError::LENGTH_DATA_READ_ERROR);
    case ReadFieldError::PARITY_BIT_READ_ERROR: return std::unexpected(MessageError::LENGTH_PARITY_BIT_READ_ERROR);
    case ReadFieldError::PARITY_WRONG: return std::unexpected(MessageError::LENGTH_PARITY_WRONG);
    }
  }
  auto const lengthData = expectedLengthData.value();

  message.length = ((lengthData == 0) ? 256 : static_cast<Size>(lengthData));

  // DATA
  for (auto& messageData : message.data | std::views::take(message.length)) {
    auto const expectedData = readField(DATA_BITS_SIZE, isTargeted);
    if (not expectedData.has_value()) {
      auto const error = expectedControlData.error();
      switch (error) {
      case ReadFieldError::DATA_READ_ERROR: return std::unexpected(MessageError::DATA_READ_ERROR);
      case ReadFieldError::PARITY_BIT_READ_ERROR: return std::unexpected(MessageError::DATA_PARITY_BIT_READ_ERROR);
      case ReadFieldError::PARITY_WRONG: return std::unexpected(MessageError::DATA_PARITY_WRONG);
      }
    }
    auto const data = expectedData.value();

    messageData = static_cast<Byte>(data);
  }

  return message;
}

auto Controller::readField(Size const bitSize, bool const sendAck) const noexcept -> std::expected<Data, ReadFieldError> {
  auto const optionalData = m_driver.readBits(bitSize);
  if (not optionalData.has_value()) {
    return std::unexpected(ReadFieldError::DATA_READ_ERROR);
  }
  auto const data = optionalData.value();

  auto const optionalParity = m_driver.readBits(1);
  if (not optionalParity.has_value()) {
    return std::unexpected(ReadFieldError::PARITY_BIT_READ_ERROR);
  }
  auto const parity = optionalParity.value();

  auto const isValidParity = checkParity(data, parity);

  if (sendAck) {
    auto const ack = (isValidParity ? AckType::ACK : AckType::NAK);

    m_driver.writeBits(static_cast<Data>(ack), 1);
  } else {
    std::ignore = m_driver.readBits(1);
  }

  if (not isValidParity) {
    return std::unexpected(ReadFieldError::PARITY_WRONG);
  }

  return data;
}

auto Controller::writeMessage(Message const& message) const noexcept -> std::expected<std::monostate, MessageError> {
  if (not m_driver.isEnabled()) {
    return std::unexpected(MessageError::CONTROLLER_DISABLED);
  }

  if (not m_driver.isBusFree()) {
    return std::unexpected(MessageError::BUS_BUSY);
  }

  auto const isTargeted = (message.broadcast == BroadcastType::FOR_DEVICE);

  // START_BIT
  m_driver.writeStartBit();

  // BROADCAST
  m_driver.writeBits(static_cast<Data>(message.broadcast), 1);

  // MASTER
  m_driver.writeBits(static_cast<Data>(message.master), ADDRESS_BITS_SIZE);
  m_driver.writeBits(static_cast<Data>(calculateParity(message.master)), 1);

  // SLAVE
  auto const expectedWriteSlaveResult = writeField(static_cast<Data>(message.slave), ADDRESS_BITS_SIZE, isTargeted);
  if (not expectedWriteSlaveResult.has_value()) {
    auto const error = expectedWriteSlaveResult.error();
    switch (error) {
    case WriteFieldError::ACK_BIT_READ_ERROR: return std::unexpected(MessageError::SLAVE_ADDRESS_ACK_BIT_READ_ERROR);
    case WriteFieldError::ACK_WRONG: return std::unexpected(MessageError::SLAVE_ADDRESS_ACK_WRONG);
    }
  }

  // CONTROL
  auto const expectedWriteControlResult = writeField(static_cast<Data>(message.control), CONTROL_BITS_SIZE, isTargeted);
  if (not expectedWriteControlResult.has_value()) {
    auto const error = expectedWriteControlResult.error();
    switch (error) {
    case WriteFieldError::ACK_BIT_READ_ERROR: return std::unexpected(MessageError::CONTROL_ACK_BIT_READ_ERROR);
    case WriteFieldError::ACK_WRONG: return std::unexpected(MessageError::CONTROL_ACK_WRONG);
    }
  }

  // LENGTH
  auto const expectedWriteLengthResult = writeField(static_cast<Data>(message.length), LENGTH_BITS_SIZE, isTargeted);
  if (not expectedWriteLengthResult.has_value()) {
    auto const error = expectedWriteLengthResult.error();
    switch (error) {
    case WriteFieldError::ACK_BIT_READ_ERROR: return std::unexpected(MessageError::LENGTH_ACK_BIT_READ_ERROR);
    case WriteFieldError::ACK_WRONG: return std::unexpected(MessageError::LENGTH_ACK_WRONG);
    }
  }

  // DATA
  for (auto const& messageData : message.data | std::views::take(message.length)) {
    auto const expectedWriteDataResult = writeField(static_cast<Data>(messageData), DATA_BITS_SIZE, isTargeted);
    if (not expectedWriteDataResult.has_value()) {
      auto const error = expectedWriteDataResult.error();
      switch (error) {
      case WriteFieldError::ACK_BIT_READ_ERROR: return std::unexpected(MessageError::DATA_ACK_BIT_READ_ERROR);
      case WriteFieldError::ACK_WRONG: return std::unexpected(MessageError::DATA_ACK_WRONG);
      }
    }
  }

  return {};
}

auto Controller::writeField(Data const data, Size const bitSize, bool forDevice) const noexcept -> std::expected<std::monostate, WriteFieldError> {
  m_driver.writeBits(data, bitSize);
  m_driver.writeBits(static_cast<Data>(calculateParity(data)), 1);

  if (forDevice) {
    auto const optionalAckBit = m_driver.readBits(1);
    if (not optionalAckBit.has_value()) {
      return std::unexpected(WriteFieldError::ACK_BIT_READ_ERROR);
    }
    auto const ackBit = optionalAckBit.value();

    if (static_cast<AckType>(ackBit) == AckType::NAK) {
      return std::unexpected(WriteFieldError::ACK_WRONG);
    }

  } else {
    m_driver.writeBits(static_cast<Data>(AckType::NAK), 1);
  }

  return std::monostate{};
}

} // namespace iebus
