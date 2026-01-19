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

  return calculatedParity == parity;
}

} // namespace

Controller::Controller(Driver const& driver, Address const address) noexcept : m_address(address), m_driver(driver) {
}

auto Controller::readMessage() noexcept -> std::expected<Message, MessageError> {
  if (not m_driver.isEnabled()) {
    return std::unexpected(MessageError::CONTROLLER_DISABLED);
  }

  // START_BIT
  auto const isStarted = m_driver.readStartBit();
  if (not isStarted) {
    return std::unexpected(MessageError::START_BIT_IS_FALSE);
  }

  Message message = {
//      BroadcastType::BROADCAST, 0x130, 0xFFF, ControlType::WRITE_COMMAND, 3, {0x10, 0x1A, 0x01}
//      BroadcastType::BROADCAST, 0x130, 0xFFF, ControlType::WRITE_COMMAND, 12, {0x60, 0xC0, 0x00, 0x4F, 0xFF, 0x00, 0x00, 0x22, 0x00, 0x22, 0x00, 0x00}
  };

//  return message;

  // BROADCAST
  auto const broadcastBit = m_driver.readBits(1);

  message.broadcast = static_cast<BroadcastType>(broadcastBit);

  // MASTER
  auto const masterData          = m_driver.readBits(ADDRESS_BITS_SIZE);
  auto const masterParityBit     = m_driver.readBits(1);
  auto const isValidMasterParity = checkParity(masterData, masterParityBit);
  if (not isValidMasterParity) {
    return std::unexpected(MessageError::MASTER_ADDRESS_PARITY_WRONG);
  }
  message.master = static_cast<Address>(masterData);

  // SLAVE
  auto const slaveData          = m_driver.readBits(ADDRESS_BITS_SIZE);
  auto const slaveParityBit     = m_driver.readBits(1);
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
  auto const optionalControlData = readVerifiedField(CONTROL_BITS_SIZE, isTargeted);
  if (not optionalControlData.has_value()) {
    return std::unexpected(MessageError::CONTROL_PARITY_WRONG);
  }
  auto const controlData = optionalControlData.value();

  message.control = static_cast<ControlType>(controlData);

  // LENGTH
  auto const optionalLengthData = readVerifiedField(LENGTH_BITS_SIZE, isTargeted);
  if (not optionalLengthData.has_value()) {
    return std::unexpected(MessageError::LENGTH_PARITY_WRONG);
  }
  auto const lengthData = optionalLengthData.value();

  message.length = (lengthData == 0) ? 256 : static_cast<Size>(lengthData);

  // DATA
  for (Size i = 0; i < message.length; ++i) {
    auto const optionalData = readVerifiedField(DATA_BITS_SIZE, isTargeted);
    if (not optionalData.has_value()) {
      return std::unexpected(MessageError::DATA_PARITY_WRONG);
    }
    auto const data = optionalData.value();

    message.data[i] = static_cast<Byte>(data);
  }

  return message;
}

auto Controller::writeMessage(Message const& message) noexcept -> bool {
  if (not m_driver.isEnabled()) {
    return false;
  }

  while (not m_driver.isBusFree()) {}

  // START_BIT
  m_driver.writeStartBit();

  // BROADCAST
  m_driver.writeBits(static_cast<Data>(message.broadcast), 1);

  // MASTER
  m_driver.writeBits(message.master, ADDRESS_BITS_SIZE);
  m_driver.writeBits(calculateParity(message.master), 1);

  // SLAVE
  m_driver.writeBits(message.slave, ADDRESS_BITS_SIZE);
  m_driver.writeBits(calculateParity(message.slave), 1);

  if (message.broadcast == BroadcastType::FOR_DEVICE) {
    auto const ackBit = m_driver.readBits(1);
    if (static_cast<AckType>(ackBit) == AckType::NAK) {
      return false;
    }

  } else {
    m_driver.writeBits(static_cast<Data>(AckType::NAK), 1);
  }

  // CONTROL
  m_driver.writeBits(static_cast<Data>(message.control), CONTROL_BITS_SIZE);
  m_driver.writeBits(calculateParity(static_cast<Data>(message.control)), 1);

  if (message.broadcast == BroadcastType::FOR_DEVICE) {
    auto const ackBit = m_driver.readBits(1);
    if (static_cast<AckType>(ackBit) == AckType::NAK) {
      return false;
    }

  } else {
    m_driver.writeBits(static_cast<Data>(AckType::NAK), 1);
  }

  // LENGTH
  m_driver.writeBits(message.length, LENGTH_BITS_SIZE);
  m_driver.writeBits(calculateParity(message.length), 1);

  if (message.broadcast == BroadcastType::FOR_DEVICE) {
    auto const ackBit = m_driver.readBits(1);
    if (static_cast<AckType>(ackBit) == AckType::NAK) {
      return false;
    }

  } else {
    m_driver.writeBits(static_cast<Data>(AckType::NAK), 1);
  }

  for (Size i = 0; i < message.length; i++) {
    m_driver.writeBits(message.data[i], DATA_BITS_SIZE);
    m_driver.writeBits(calculateParity(message.data[i]), 1);

    if (message.broadcast == BroadcastType::FOR_DEVICE) {
      auto const ackBit = m_driver.readBits(1);
      if (static_cast<AckType>(ackBit) == AckType::NAK) {
        return false;
      }

    } else {
      m_driver.writeBits(static_cast<Data>(AckType::NAK), 1);
    }
  }

  m_driver.waitBusBusy();

  return true;
}

auto Controller::readVerifiedField(Size const bitSize, bool const sendAck) noexcept -> std::optional<Data> {
  auto const data          = m_driver.readBits(bitSize);
  auto const parity        = m_driver.readBits(1);
  auto const isValidParity = checkParity(data, parity);

  if (sendAck) {
    auto const ack = isValidParity ? AckType::ACK : AckType::NAK;
    m_driver.writeBits(static_cast<Data>(ack), 1);
  } else {
    std::ignore = m_driver.readBits(1);
  }

  if (not isValidParity) {
    return std::nullopt;
  }

  return data;
}

} // namespace iebus
