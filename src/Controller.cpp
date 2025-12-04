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
#include <esp_timer.h>
#include <utility>

#include "common.hpp"

namespace iebus {

namespace {

auto constexpr TAG = "IEBusController";

auto constexpr MASTER_ADDRESS_BIT_SIZE = 12;
auto constexpr SLAVE_ADDRESS_BIT_SIZE = 12;
auto constexpr CONTROL_BIT_SIZE = 4;
auto constexpr DATA_LENGTH_BIT_SIZE = 8;
auto constexpr DATA_BIT_SIZE = 8;

} // namespace

Controller::Controller(Driver::Pin const rx, Driver::Pin const tx, Driver::Pin const enable, Address const address) noexcept : m_address(address), m_driver(rx, tx, enable) {}

auto Controller::enable() -> void { m_driver.enable(); }

auto Controller::disable() -> void { m_driver.disable(); }

auto Controller::isEnabled() const -> bool { return m_driver.isEnabled(); }

auto Controller::readMessage() const -> std::optional<Message> {
  if (not isEnabled()) {
    ESP_LOGE(TAG, "Controller is disabled");
    return std::nullopt;
  }

  if (not m_driver.receiveStartBit()) {
    return std::nullopt;
  }

  Message message = {};

  {
    auto const broadcastBit = m_driver.receiveBit();
    if (broadcastBit == 0) {
      message.broadcast = BroadcastType::BROADCAST;
    } else {
      message.broadcast = BroadcastType::FOR_DEVICE;
    }
  }

  {
    message.master = m_driver.receiveBits(MASTER_ADDRESS_BIT_SIZE);

    auto const masterParityBit = m_driver.receiveBit();
    auto const isParityValid = checkParity(message.master, MASTER_ADDRESS_BIT_SIZE, masterParityBit);
    if (not isParityValid) {
      ESP_LOGW(TAG, "Master address parity error");
      return std::nullopt;
    }
  }

  {
    message.slave = m_driver.receiveBits(SLAVE_ADDRESS_BIT_SIZE);

    auto const parityBit = m_driver.receiveBit();
    auto const isParityValid = checkParity(message.slave, SLAVE_ADDRESS_BIT_SIZE, parityBit);
    auto const ackBit = m_driver.receiveAckBit();
    auto const isNeedAnswer = ackBit == AcknowledgmentType::ACK;
    auto const isForDevice = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.sendAckBit(AcknowledgmentType::NAK);
      }

      ESP_LOGW(TAG, "Slave address parity error");
      return std::nullopt;
    }

    if (isAnswer) {
      m_driver.sendAckBit(AcknowledgmentType::ACK);
    }
  }

  {
    message.control = m_driver.receiveBits(CONTROL_BIT_SIZE);

    auto const parityBit = m_driver.receiveBit();
    auto const isParityValid = checkParity(message.control, CONTROL_BIT_SIZE, parityBit);
    auto const ackBit = m_driver.receiveAckBit();
    auto const isNeedAnswer = ackBit == AcknowledgmentType::ACK;
    auto const isForDevice = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.sendAckBit(AcknowledgmentType::NAK);
      }

      ESP_LOGW(TAG, "Control parity error");
      return std::nullopt;
    }

    if (isAnswer) {
      m_driver.sendAckBit(AcknowledgmentType::ACK);
    }
  }

  {
    message.dataLength = m_driver.receiveBits(DATA_LENGTH_BIT_SIZE);

    auto const parityBit = m_driver.receiveBit();
    auto const isParityValid = checkParity(message.dataLength, DATA_LENGTH_BIT_SIZE, parityBit);
    auto const ackBit = m_driver.receiveAckBit();
    auto const isNeedAnswer = ackBit == AcknowledgmentType::ACK;
    auto const isForDevice = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.sendAckBit(AcknowledgmentType::NAK);
      }

      ESP_LOGW(TAG, "Length parity error");
      return std::nullopt;
    }

    if (isAnswer) {
      m_driver.sendAckBit(AcknowledgmentType::ACK);
    }

    if (message.dataLength == 0) {
      message.dataLength = 256;
    }
  }

  for (Size i = 0; i < message.dataLength; i++) {
    message.data[i] = m_driver.receiveBits(DATA_BIT_SIZE);

    auto const parityBit = m_driver.receiveBit();
    auto const isParityValid = checkParity(message.data[i], DATA_BIT_SIZE, parityBit);
    auto const ackBit = m_driver.receiveAckBit();
    auto const isNeedAnswer = ackBit == AcknowledgmentType::ACK;
    auto const isForDevice = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isForThisDevice = message.slave == m_address;
    auto const isAnswer = isNeedAnswer and isForDevice and isForThisDevice;

    if (not isParityValid) {
      if (isAnswer) {
        m_driver.sendAckBit(AcknowledgmentType::NAK);
      }

      ESP_LOGW(TAG, "Data byte %u parity error", i);
      return std::nullopt;
    }

    if (isAnswer) {
      m_driver.sendAckBit(AcknowledgmentType::ACK);
    }
  }

  return message;
}

auto Controller::writeMessage(Message const &message) const -> bool {
  if (not isEnabled()) {
    ESP_LOGE(TAG, "Controller is disabled");
    return false;
  }

  while (not m_driver.isBusFree()) {
    delayUs(1);
  }

  m_driver.transmitStartBit();

  if (message.broadcast == BroadcastType::BROADCAST) {
    m_driver.transmitBit(0);
  } else {
    m_driver.transmitBit(1);
  }

  {
    m_driver.transmitBits(message.master, 12);

    auto const parityBit = calculateParity(message.master, 12);
    m_driver.transmitBit(parityBit);
  }

  {
    m_driver.transmitBits(message.slave, 12);

    auto const parityBit = calculateParity(message.slave, 12);
    m_driver.transmitBit(parityBit);

    auto const ackBit = m_driver.receiveAckBit();
    if (ackBit == AcknowledgmentType::NAK) {
      ESP_LOGE(TAG, "No ACK for address");
      return false;
    }
  }

  {
    m_driver.transmitBits(message.control, 4);

    auto const parityBit = calculateParity(message.control, 4);
    m_driver.transmitBit(parityBit);

    auto const ackBit = m_driver.receiveAckBit();
    if (ackBit == AcknowledgmentType::NAK) {
      ESP_LOGE(TAG, "No ACK for control");
      return false;
    }
  }

  {
    m_driver.transmitBits(message.dataLength, 8);

    auto const parityBit = calculateParity(message.dataLength, 8);
    m_driver.transmitBit(parityBit);

    auto const ackBit = m_driver.receiveAckBit();
    if (ackBit == AcknowledgmentType::NAK) {
      ESP_LOGE(TAG, "No ACK for data length");
      return false;
    }
  }

  for (Size i = 0; i < message.dataLength; i++) {
    m_driver.transmitBits(message.data[i], 8);

    auto const parityBit = calculateParity(message.data[i], 8);
    m_driver.transmitBit(parityBit);

    auto const ackBit = m_driver.receiveAckBit();
    if (ackBit == AcknowledgmentType::NAK) {
      ESP_LOGE(TAG, "No ACK for data byte %u", i);
      return false;
    }
  }

  return true;
}

auto Controller::checkParity(Data const data, Size const size, Bit const parity) -> bool { return calculateParity(data, size) == parity; }

auto Controller::calculateParity(Data const data, Size const size) -> Bit {
  Bit parity = 0;

  for (auto i = 0; std::cmp_less(i, size); i++) {
    parity ^= data >> i & 1;
  }

  return parity;
}

} // namespace iebus
