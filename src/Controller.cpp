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

#include <esp_log.h>

#include "common.hpp"
#include "iebus/Message.hpp"

namespace iebus {

namespace {

auto constexpr TAG = "IEBusController";

auto constexpr MASTER_ADDRESS_BIT_SIZE = 12;
auto constexpr SLAVE_ADDRESS_BIT_SIZE  = 12;
auto constexpr CONTROL_BIT_SIZE        = 4;
auto constexpr DATA_LENGTH_BIT_SIZE    = 8;
auto constexpr DATA_BIT_SIZE           = 8;

auto calculateParity(Data const data, Size const size) -> Bit {
  Bit parity = 0;

  for (Size i = 0; i < size; i++) {
    parity ^= data >> i & 1;
  }

  return parity;
}

auto checkParity(Data const data, Size const size, Bit const parity) -> bool {
  auto const calculatedParity = calculateParity(data, size);

  return calculatedParity == parity;
}

} // namespace

Controller::Controller(Pin const rx, Pin const tx, Pin const enable, Address const address) noexcept : m_address(address), m_driver(rx, tx, enable) {
}

auto Controller::enable() const -> void {
  m_driver.enable();
}

auto Controller::disable() const -> void {
  m_driver.disable();
}

auto Controller::isEnabled() const -> bool {
  return m_driver.isEnabled();
}

auto Controller::registerOnMaster() const -> bool {
  Message const message = {
      .broadcast = BroadcastType::BROADCAST,
      .master    = m_address,
      .slave     = 0xFFF,
      .control   = 0xF,
      .length    = 1,
      .data      = {0x1F},
  };

  auto const expectedResult = writeMessage(message);
  if (expectedResult.has_value()) {
    return true;
  }

  return false;
}

auto Controller::readMessage() const -> std::expected<Message, MessageError> {
  if (not isEnabled()) {
    return std::unexpected(MessageError::CONTROLLER_DISABLED);
  }

  if (not m_driver.readStartBit()) {
    return std::unexpected(MessageError::START_BIT_READ_ERROR);
  }

  Message message = {};

  {
    auto const optionalBroadcastBit = m_driver.readBit();
    if (not optionalBroadcastBit) {
      return std::unexpected(MessageError::BROADCAST_BIT_READ_ERROR);
    }

    auto const broadcastBit = optionalBroadcastBit.value();

    message.broadcast = static_cast<BroadcastType>(broadcastBit);
  }

  {
    auto const optionalData = m_driver.readBits(MASTER_ADDRESS_BIT_SIZE);
    if (not optionalData) {
      return std::unexpected(MessageError::MASTER_ADDRESS_DATA_READ_ERROR);
    }

    auto const optionalParityBit = m_driver.readBit();
    if (not optionalParityBit) {
      return std::unexpected(MessageError::MASTER_ADDRESS_PARITY_BIT_READ_ERROR);
    }

    auto const data      = optionalData.value();
    auto const parityBit = optionalParityBit.value();

    message.master = static_cast<Address>(data);

    auto const isParityValid = checkParity(data, MASTER_ADDRESS_BIT_SIZE, parityBit);
    if (not isParityValid) {
      return std::unexpected(MessageError::MASTER_ADDRESS_PARITY_WRONG);
    }
  }

  {
    auto const optionalData = m_driver.readBits(SLAVE_ADDRESS_BIT_SIZE);
    if (not optionalData) {
      return std::unexpected(MessageError::SLAVE_ADDRESS_DATA_READ_ERROR);
    }

    auto const optionalParityBit = m_driver.readBit();
    if (not optionalParityBit) {
      return std::unexpected(MessageError::SLAVE_ADDRESS_PARITY_BIT_READ_ERROR);
    }

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageError::SLAVE_ADDRESS_ACK_BIT_READ_ERROR);
    }

    auto const data      = optionalData.value();
    auto const parityBit = optionalParityBit.value();
    auto const ackBit    = optionalAckBit.value();

    message.slave = static_cast<Address>(data);

    auto const isParityValid   = checkParity(data, SLAVE_ADDRESS_BIT_SIZE, parityBit);
    auto const isNeedAnswer    = ackBit == AckType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(AckType::NAK);
      }

      return std::unexpected(MessageError::SLAVE_ADDRESS_PARITY_WRONG);
    }

    if (isAnswer) {
      m_driver.writeAckBit(AckType::ACK);
    }
  }

  {
    auto const optionalData = m_driver.readBits(CONTROL_BIT_SIZE);
    if (not optionalData) {
      return std::unexpected(MessageError::CONTROL_DATA_READ_ERROR);
    }

    auto const optionalParityBit = m_driver.readBit();
    if (not optionalParityBit) {
      return std::unexpected(MessageError::CONTROL_PARITY_BIT_READ_ERROR);
    }

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageError::CONTROL_ACK_BIT_READ_ERROR);
    }

    auto const data      = optionalData.value();
    auto const parityBit = optionalParityBit.value();
    auto const ackBit    = optionalAckBit.value();

    message.control = static_cast<Byte>(data);

    auto const isParityValid   = checkParity(data, CONTROL_BIT_SIZE, parityBit);
    auto const isNeedAnswer    = ackBit == AckType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(AckType::NAK);
      }

      return std::unexpected(MessageError::CONTROL_PARITY_WRONG);
    }

    if (isAnswer) {
      m_driver.writeAckBit(AckType::ACK);
    }
  }

  {
    auto const optionalData = m_driver.readBits(DATA_LENGTH_BIT_SIZE);
    if (not optionalData) {
      return std::unexpected(MessageError::LENGTH_DATA_READ_ERROR);
    }

    auto const optionalParityBit = m_driver.readBit();
    if (not optionalParityBit) {
      return std::unexpected(MessageError::LENGTH_PARITY_BIT_READ_ERROR);
    }

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageError::LENGTH_ACK_BIT_READ_ERROR);
    }

    auto const data      = optionalData.value();
    auto const parityBit = optionalParityBit.value();
    auto const ackBit    = optionalAckBit.value();

    if (data == 0) {
      message.length = static_cast<Size>(256);
    } else {
      message.length = static_cast<Size>(data);
    }

    auto const isParityValid   = checkParity(data, DATA_LENGTH_BIT_SIZE, parityBit);
    auto const isNeedAnswer    = ackBit == AckType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(AckType::NAK);
      }

      return std::unexpected(MessageError::LENGTH_PARITY_WRONG);
    }

    if (isAnswer) {
      m_driver.writeAckBit(AckType::ACK);
    }
  }

  for (Size i = 0; i < message.length; i++) {
    auto const optionalData = m_driver.readBits(DATA_BIT_SIZE);
    if (not optionalData) {
      return std::unexpected(MessageError::DATA_READ_ERROR);
    }

    auto const optionalParityBit = m_driver.readBit();
    if (not optionalParityBit) {
      return std::unexpected(MessageError::DATA_PARITY_BIT_READ_ERROR);
    }

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageError::DATA_ACK_BIT_READ_ERROR);
    }

    auto const data      = optionalData.value();
    auto const parityBit = optionalParityBit.value();
    auto const ackBit    = optionalAckBit.value();

    message.data[i] = static_cast<Byte>(data);

    auto const isParityValid   = checkParity(data, DATA_BIT_SIZE, parityBit);
    auto const isNeedAnswer    = ackBit == AckType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(AckType::NAK);
      }

      return std::unexpected(MessageError::DATA_PARITY_WRONG);
    }

    if (isAnswer) {
      m_driver.writeAckBit(AckType::ACK);
    }
  }

  return message;
}

auto Controller::writeMessage(Message const& message) const -> std::expected<bool, MessageError> {
  if (not isEnabled()) {
    return std::unexpected(MessageError::CONTROLLER_DISABLED);
  }

  while (not m_driver.isBusFree()) {
  }

  auto const isStarted = m_driver.writeStartBit();
  if (not isStarted) {
    return std::unexpected(MessageError::START_BIT_ARBITRATION_LOST);
  }

  m_driver.writeBit(static_cast<Bit>(message.broadcast));

  {
    m_driver.writeBits(message.master, MASTER_ADDRESS_BIT_SIZE);

    auto const parityBit = calculateParity(message.master, MASTER_ADDRESS_BIT_SIZE);
    m_driver.writeBit(parityBit);
  }

  {
    m_driver.writeBits(message.slave, SLAVE_ADDRESS_BIT_SIZE);

    auto const parityBit = calculateParity(message.slave, SLAVE_ADDRESS_BIT_SIZE);
    m_driver.writeBit(parityBit);

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageError::SLAVE_ADDRESS_ACK_BIT_READ_ERROR);
    }

    auto const ackBit = optionalAckBit.value();
    if (ackBit == AckType::NAK) {
      return std::unexpected(MessageError::SLAVE_ADDRESS_ACK_WRONG);
    }
  }

  {
    m_driver.writeBits(message.control, CONTROL_BIT_SIZE);

    auto const parityBit = calculateParity(message.control, CONTROL_BIT_SIZE);
    m_driver.writeBit(parityBit);

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageError::CONTROL_ACK_BIT_READ_ERROR);
    }

    auto const ackBit = optionalAckBit.value();
    if (ackBit == AckType::NAK) {
      return std::unexpected(MessageError::CONTROL_ACK_WRONG);
    }
  }

  {
    m_driver.writeBits(message.length, DATA_LENGTH_BIT_SIZE);

    auto const parityBit = calculateParity(message.length, DATA_LENGTH_BIT_SIZE);
    m_driver.writeBit(parityBit);

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageError::LENGTH_ACK_BIT_READ_ERROR);
    }

    auto const ackBit = optionalAckBit.value();
    if (ackBit == AckType::NAK) {
      return std::unexpected(MessageError::LENGTH_ACK_WRONG);
    }
  }

  for (Size i = 0; i < message.length; i++) {
    m_driver.writeBits(message.data[i], DATA_BIT_SIZE);

    auto const parityBit = calculateParity(message.data[i], DATA_BIT_SIZE);
    m_driver.writeBit(parityBit);

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageError::DATA_ACK_BIT_READ_ERROR);
    }

    auto const ackBit = optionalAckBit.value();
    if (ackBit == AckType::NAK) {
      return std::unexpected(MessageError::DATA_ACK_WRONG);
    }
  }

  return true;
}

} // namespace iebus
