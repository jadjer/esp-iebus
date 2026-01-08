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
// Created by jadjer on 29.11.2025.
//

#include "iebus/Controller.hpp"

#include <esp_log.h>

#include "common.hpp"

namespace iebus {

namespace {

auto constexpr TAG = "IEBusController";

auto constexpr MASTER_ADDRESS_BIT_SIZE = 12;
auto constexpr SLAVE_ADDRESS_BIT_SIZE  = 12;
auto constexpr CONTROL_BIT_SIZE        = 4;
auto constexpr DATA_LENGTH_BIT_SIZE    = 8;
auto constexpr DATA_BIT_SIZE           = 8;

auto calculateParity(Driver::Data const data, Driver::Size const size) -> Driver::Bit {
  Driver::Bit parity = 0;

  for (Driver::Size i = 0; i < size; i++) {
    parity ^= data >> i & 1;
  }

  return parity;
}

auto checkParity(Driver::Data const data, Driver::Size const size, Driver::Bit const parity) -> bool {
  auto const calculatedParity = calculateParity(data, size);

  return calculatedParity == parity;
}

} // namespace

Controller::Controller(Driver::Pin const rx, Driver::Pin const tx, Driver::Pin const enable, Address const address) noexcept : m_address(address), m_driver(rx, tx, enable) {
}

auto Controller::enable() -> void {
  m_driver.enable();
}

auto Controller::disable() -> void {
  m_driver.disable();
}

auto Controller::isEnabled() const -> bool {
  return m_driver.isEnabled();
}

auto Controller::readMessage() -> std::expected<Message, MessageReadError> {
  if (not isEnabled()) {
    return std::unexpected(MessageReadError::CONTROLLER_DISABLED);
  }

  if (not m_driver.readStartBit()) {
    return std::unexpected(MessageReadError::START_BIT_READ_ERROR);
  }

  Message message = {};

  {
    auto const optionalBroadcastBit = m_driver.readBit();
    if (not optionalBroadcastBit) {
      return std::unexpected(MessageReadError::BROADCAST_BIT_READ_ERROR);
    }

    auto const broadcastBit = optionalBroadcastBit.value();

    if (broadcastBit == 0) {
      message.broadcast = BroadcastType::BROADCAST;
    } else {
      message.broadcast = BroadcastType::FOR_DEVICE;
    }
  }

  {
    auto const optionalData = m_driver.readBits(MASTER_ADDRESS_BIT_SIZE);
    if (not optionalData) {
      return std::unexpected(MessageReadError::MASTER_ADDRESS_DATA_READ_ERROR);
    }

    auto const optionalParityBit = m_driver.readBit();
    if (not optionalParityBit) {
      return std::unexpected(MessageReadError::MASTER_ADDRESS_PARITY_BIT_READ_ERROR);
    }

    auto const data      = optionalData.value();
    auto const parityBit = optionalParityBit.value();

    auto const isParityValid = checkParity(data, MASTER_ADDRESS_BIT_SIZE, parityBit);
    if (not isParityValid) {
      return std::unexpected(MessageReadError::MASTER_ADDRESS_PARITY_WRONG);
    }

    message.master = data;
  }

  {
    auto const optionalData = m_driver.readBits(SLAVE_ADDRESS_BIT_SIZE);
    if (not optionalData) {
      return std::unexpected(MessageReadError::SLAVE_ADDRESS_DATA_READ_ERROR);
    }

    auto const optionalParityBit = m_driver.readBit();
    if (not optionalParityBit) {
      return std::unexpected(MessageReadError::SLAVE_ADDRESS_PARITY_BIT_READ_ERROR);
    }

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageReadError::SLAVE_ADDRESS_ACK_BIT_READ_ERROR);
    }

    auto const data            = optionalData.value();
    auto const parityBit       = optionalParityBit.value();
    auto const ackBit          = optionalAckBit.value();
    auto const isParityValid   = checkParity(data, SLAVE_ADDRESS_BIT_SIZE, parityBit);
    auto const isNeedAnswer    = ackBit == Driver::AckType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(Driver::AckType::NAK);
      }

      return std::unexpected(MessageReadError::SLAVE_ADDRESS_PARITY_WRONG);
    }

    if (isAnswer) {
      m_driver.writeAckBit(Driver::AckType::ACK);
    }

    message.slave = data;
  }

  {
    auto const optionalData = m_driver.readBits(CONTROL_BIT_SIZE);
    if (not optionalData) {
      return std::unexpected(MessageReadError::CONTROL_DATA_READ_ERROR);
    }

    auto const optionalParityBit = m_driver.readBit();
    if (not optionalParityBit) {
      return std::unexpected(MessageReadError::CONTROL_PARITY_BIT_READ_ERROR);
    }

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageReadError::CONTROL_ACK_BIT_READ_ERROR);
    }

    auto const data            = optionalData.value();
    auto const parityBit       = optionalParityBit.value();
    auto const ackBit          = optionalAckBit.value();
    auto const isParityValid   = checkParity(data, CONTROL_BIT_SIZE, parityBit);
    auto const isNeedAnswer    = ackBit == Driver::AckType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(Driver::AckType::NAK);
      }

      return std::unexpected(MessageReadError::CONTROL_PARITY_WRONG);
    }

    if (isAnswer) {
      m_driver.writeAckBit(Driver::AckType::ACK);
    }

    message.control = data;
  }

  {
    auto const optionalData = m_driver.readBits(DATA_LENGTH_BIT_SIZE);
    if (not optionalData) {
      return std::unexpected(MessageReadError::LENGTH_DATA_READ_ERROR);
    }

    auto const optionalParityBit = m_driver.readBit();
    if (not optionalParityBit) {
      return std::unexpected(MessageReadError::LENGTH_PARITY_BIT_READ_ERROR);
    }

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageReadError::LENGTH_ACK_BIT_READ_ERROR);
    }

    auto const data            = optionalData.value();
    auto const parityBit       = optionalParityBit.value();
    auto const ackBit          = optionalAckBit.value();
    auto const isParityValid   = checkParity(data, DATA_LENGTH_BIT_SIZE, parityBit);
    auto const isNeedAnswer    = ackBit == Driver::AckType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(Driver::AckType::NAK);
      }

      return std::unexpected(MessageReadError::LENGTH_PARITY_WRONG);
    }

    if (isAnswer) {
      m_driver.writeAckBit(Driver::AckType::ACK);
    }

    if (data == 0) {
      message.dataLength = 256;
    } else {
      message.dataLength = data;
    }
  }

  for (Size i = 0; i < message.dataLength; i++) {
    auto const optionalData = m_driver.readBits(DATA_BIT_SIZE);
    if (not optionalData) {
      return std::unexpected(MessageReadError::DATA_READ_ERROR);
    }

    auto const optionalParityBit = m_driver.readBit();
    if (not optionalParityBit) {
      return std::unexpected(MessageReadError::DATA_PARITY_BIT_READ_ERROR);
    }

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      return std::unexpected(MessageReadError::DATA_ACK_BIT_READ_ERROR);
    }

    auto const data            = optionalData.value();
    auto const parityBit       = optionalParityBit.value();
    auto const ackBit          = optionalAckBit.value();
    auto const isParityValid   = checkParity(data, DATA_BIT_SIZE, parityBit);
    auto const isNeedAnswer    = ackBit == Driver::AckType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(Driver::AckType::NAK);
      }

      return std::unexpected(MessageReadError::DATA_PARITY_WRONG);
    }

    if (isAnswer) {
      m_driver.writeAckBit(Driver::AckType::ACK);
    }

    message.data[i] = data;
  }

  return message;
}

auto Controller::writeMessage(Message const& message) -> bool {
  if (not isEnabled()) {
    ESP_LOGE(TAG, "Controller is disabled");
    return false;
  }

  while (not m_driver.isBusFree()) {
    delayUS(1);
  }

  m_driver.writeStartBit();

  if (message.broadcast == BroadcastType::BROADCAST) {
    m_driver.writeBit(0);
  } else {
    m_driver.writeBit(1);
  }

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
      ESP_LOGE(TAG, "Ack bit read error");
      return false;
    }

    auto const ackBit = optionalAckBit.value();

    if (ackBit == Driver::AckType::NAK) {
      ESP_LOGE(TAG, "No ACK for address");
      return false;
    }
  }

  {
    m_driver.writeBits(message.control, CONTROL_BIT_SIZE);

    auto const parityBit = calculateParity(message.control, CONTROL_BIT_SIZE);
    m_driver.writeBit(parityBit);

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      ESP_LOGE(TAG, "Ack bit read error");
      return false;
    }

    auto const ackBit = optionalAckBit.value();

    if (ackBit == Driver::AckType::NAK) {
      ESP_LOGE(TAG, "No ACK for control");
      return false;
    }
  }

  {
    m_driver.writeBits(message.dataLength, DATA_LENGTH_BIT_SIZE);

    auto const parityBit = calculateParity(message.dataLength, DATA_LENGTH_BIT_SIZE);
    m_driver.writeBit(parityBit);

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      ESP_LOGE(TAG, "Ack bit read error");
      return false;
    }

    auto const ackBit = optionalAckBit.value();

    if (ackBit == Driver::AckType::NAK) {
      ESP_LOGE(TAG, "No ACK for data length");
      return false;
    }
  }

  for (Size i = 0; i < message.dataLength; i++) {
    m_driver.writeBits(message.data[i], DATA_BIT_SIZE);

    auto const parityBit = calculateParity(message.data[i], DATA_BIT_SIZE);
    m_driver.writeBit(parityBit);

    auto const optionalAckBit = m_driver.readAckBit();
    if (not optionalAckBit) {
      ESP_LOGE(TAG, "Ack bit read error");
      return false;
    }

    auto const ackBit = optionalAckBit.value();

    if (ackBit == Driver::AckType::NAK) {
      ESP_LOGE(TAG, "No ACK for data byte %u", i);
      return false;
    }
  }

  return true;
}

} // namespace iebus
