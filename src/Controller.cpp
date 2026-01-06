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

auto Controller::readMessage() -> std::optional<Message> {
  if (not isEnabled()) {
    ESP_LOGE(TAG, "Controller is disabled");
    return std::nullopt;
  }

  if (not m_driver.readStartBit()) {
    return std::nullopt;
  }

  Message message = {};

  {
    auto const broadcastBit = m_driver.readBit();
    if (not broadcastBit) {
      ESP_LOGE(TAG, "Broadcast bit read error");
      return std::nullopt;
    }

    if (broadcastBit.value() == 0) {
      message.broadcast = BroadcastType::BROADCAST;
    } else {
      message.broadcast = BroadcastType::FOR_DEVICE;
    }
  }

  {
    auto const data = m_driver.readBits(MASTER_ADDRESS_BIT_SIZE);
    if (not data) {
      ESP_LOGE(TAG, "Master address data read error");
      return std::nullopt;
    }

    message.master = data.value();

    auto const parityBit = m_driver.readBit();
    if (not parityBit) {
      ESP_LOGW(TAG, "Master address parity bit read error");
      return std::nullopt;
    }

    auto const isParityValid = checkParity(message.master, MASTER_ADDRESS_BIT_SIZE, parityBit.value());
    if (not isParityValid) {
      ESP_LOGW(TAG, "Master address parity error");
      return std::nullopt;
    }
  }

  {
    auto const data = m_driver.readBits(SLAVE_ADDRESS_BIT_SIZE);
    if (not data) {
      ESP_LOGE(TAG, "Slave address data read error");
      return std::nullopt;
    }

    auto const parity = m_driver.readBit();
    if (not parity) {
      ESP_LOGW(TAG, "Slave address parity bit read error");
      return std::nullopt;
    }

    auto const ack = m_driver.readAckBit();
    if (not ack) {
      ESP_LOGE(TAG, "Slave address ack bit read error");
      return std::nullopt;
    }

    message.slave = data.value();

    auto const isParityValid   = checkParity(message.slave, SLAVE_ADDRESS_BIT_SIZE, parity.value());
    auto const isNeedAnswer    = ack.value() == AcknowledgmentType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(AcknowledgmentType::NAK);
      }

      ESP_LOGW(TAG, "Slave address parity error");
      return std::nullopt;
    }

    if (isAnswer) {
      m_driver.writeAckBit(AcknowledgmentType::ACK);
    }
  }

  {
    auto const data = m_driver.readBits(CONTROL_BIT_SIZE);
    if (not data) {
      ESP_LOGE(TAG, "Control data read error");
      return std::nullopt;
    }

    auto const parity = m_driver.readBit();
    if (not parity) {
      ESP_LOGW(TAG, "Control parity bit read error");
      return std::nullopt;
    }

    auto const ack = m_driver.readAckBit();
    if (not ack) {
      ESP_LOGE(TAG, "Control ack bit read error");
      return std::nullopt;
    }

    message.control = data.value();

    auto const isParityValid   = checkParity(message.control, CONTROL_BIT_SIZE, parity.value());
    auto const isNeedAnswer    = ack.value() == AcknowledgmentType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(AcknowledgmentType::NAK);
      }

      ESP_LOGW(TAG, "Control parity error");
      return std::nullopt;
    }

    if (isAnswer) {
      m_driver.writeAckBit(AcknowledgmentType::ACK);
    }
  }

  {
    auto const data = m_driver.readBits(DATA_LENGTH_BIT_SIZE);
    if (not data) {
      ESP_LOGE(TAG, "Data length read error");
      return std::nullopt;
    }

    auto const parity = m_driver.readBit();
    if (not parity) {
      ESP_LOGW(TAG, "Data length parity bit read error");
      return std::nullopt;
    }

    auto const ack = m_driver.readAckBit();
    if (not ack) {
      ESP_LOGE(TAG, "Data length ack bit read error");
      return std::nullopt;
    }

    message.dataLength = data.value();

    auto const isParityValid   = checkParity(message.dataLength, DATA_LENGTH_BIT_SIZE, parity.value());
    auto const isNeedAnswer    = ack.value() == AcknowledgmentType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(AcknowledgmentType::NAK);
      }

      ESP_LOGW(TAG, "Length parity error");
      return std::nullopt;
    }

    if (isAnswer) {
      m_driver.writeAckBit(AcknowledgmentType::ACK);
    }

    if (message.dataLength == 0) {
      message.dataLength = 256;
    }
  }

  for (Size i = 0; i < message.dataLength; i++) {
    auto const data = m_driver.readBits(DATA_BIT_SIZE);
    if (not data) {
      ESP_LOGE(TAG, "Data[%u] read error", i);
      return std::nullopt;
    }

    auto const parity = m_driver.readBit();
    if (not parity) {
      ESP_LOGW(TAG, "Data[%u] parity bit read error", i);
      return std::nullopt;
    }

    auto const ack        = m_driver.readAckBit();
    if (not ack) {
      ESP_LOGE(TAG, "Data[%u] ack bit read error", i);
      return std::nullopt;
    }

    message.data[i] = data.value();

    auto const isParityValid = checkParity(message.data[i], DATA_BIT_SIZE, parity.value());
    auto const isNeedAnswer    = ack.value() == AcknowledgmentType::ACK;
    auto const isForDevice     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer        = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.writeAckBit(AcknowledgmentType::NAK);
      }

      ESP_LOGW(TAG, "Data[&u] byte parity error", i);
      return std::nullopt;
    }

    if (isAnswer) {
      m_driver.writeAckBit(AcknowledgmentType::ACK);
    }
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
    m_driver.writeBits(message.master, 12);

    auto const parityBit = calculateParity(message.master, 12);
    m_driver.writeBit(parityBit);
  }

  {
    m_driver.writeBits(message.slave, 12);

    auto const parityBit = calculateParity(message.slave, 12);
    m_driver.writeBit(parityBit);

    auto const ackBit = m_driver.readAckBit();
    if (not ackBit) {
      ESP_LOGE(TAG, "Ack bit read error");
      return false;
    }
    if (ackBit.value() == AcknowledgmentType::NAK) {
      ESP_LOGE(TAG, "No ACK for address");
      return false;
    }
  }

  {
    m_driver.writeBits(message.control, 4);

    auto const parityBit = calculateParity(message.control, 4);
    m_driver.writeBit(parityBit);

    auto const ackBit = m_driver.readAckBit();
    if (not ackBit) {
      ESP_LOGE(TAG, "Ack bit read error");
      return false;
    }
    if (ackBit.value() == AcknowledgmentType::NAK) {
      ESP_LOGE(TAG, "No ACK for control");
      return false;
    }
  }

  {
    m_driver.writeBits(message.dataLength, 8);

    auto const parityBit = calculateParity(message.dataLength, 8);
    m_driver.writeBit(parityBit);

    auto const ackBit = m_driver.readAckBit();
    if (not ackBit) {
      ESP_LOGE(TAG, "Ack bit read error");
      return false;
    }
    if (ackBit.value() == AcknowledgmentType::NAK) {
      ESP_LOGE(TAG, "No ACK for data length");
      return false;
    }
  }

  for (Size i = 0; i < message.dataLength; i++) {
    m_driver.writeBits(message.data[i], 8);

    auto const parityBit = calculateParity(message.data[i], 8);
    m_driver.writeBit(parityBit);

    auto const ackBit = m_driver.readAckBit();
    if (not ackBit) {
      ESP_LOGE(TAG, "Ack bit read error");
      return false;
    }
    if (ackBit.value() == AcknowledgmentType::NAK) {
      ESP_LOGE(TAG, "No ACK for data byte %u", i);
      return false;
    }
  }

  return true;
}

auto Controller::checkParity(Driver::Data const data, Size const size, Bit const parity) -> bool {
  auto const calculatedParity = calculateParity(data, size);

  auto const isValid = calculatedParity == parity;
  if (not isValid) {
    ESP_LOGE(TAG, "Parity error (%d != %d)", calculatedParity, parity);
    return false;
  }

  return true;
}

auto Controller::calculateParity(Driver::Data const data, Size const size) -> Bit {
  Bit parity = 0;

  for (Size i = 0; i < size; i++) {
    parity ^= data >> i & 1;
  }

  return parity;
}

} // namespace iebus
