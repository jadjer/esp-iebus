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

#include "common.hpp"
#include "iebus/Message.hpp"

namespace iebus {

namespace {

auto constexpr TAG = "IEBusController";

auto constexpr ADDRESS_BIT_SIZE = 12;
auto constexpr CONTROL_BIT_SIZE = 4;
auto constexpr LENGTH_BIT_SIZE  = 8;
auto constexpr DATA_BIT_SIZE    = 8;

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

Controller::Controller(Driver const& driver, Address const address) noexcept : m_driver(driver), m_address(address) {
}

auto Controller::readMessage() const noexcept -> std::optional<Message> {
  if (not m_driver.isEnabled() or not m_driver.readStartBit()) {
    return std::nullopt;
  }

  Message message{};

  // BROADCAST
  auto const optionalBroadcast = m_driver.readBit();
  if (not optionalBroadcast) {
    return std::nullopt;
  }
  message.broadcast = static_cast<BroadcastType>(*optionalBroadcast);

  // MASTER
  auto const optionalMaster       = m_driver.readBits(ADDRESS_BIT_SIZE);
  auto const optionalMasterParity = m_driver.readBit();
  if (not optionalMaster or not optionalMasterParity) {
    return std::nullopt;
  }
  if (not checkParity(*optionalMaster, ADDRESS_BIT_SIZE, *optionalMasterParity)) {
    return std::nullopt;
  }
  message.master = static_cast<Address>(*optionalMaster);

  // SLAVE
  auto const optionalSlave       = m_driver.readBits(ADDRESS_BIT_SIZE);
  auto const optionalSlaveParity = m_driver.readBit();
  if (not optionalSlave or not optionalSlaveParity) {
    return std::nullopt;
  }
  message.slave = static_cast<Address>(*optionalSlave);

  bool const isTargeted    = (message.broadcast == BroadcastType::FOR_DEVICE and message.slave == m_address);
  bool const slaveParityOk = checkParity(*optionalSlave, ADDRESS_BIT_SIZE, *optionalSlaveParity);

  if (isTargeted) {
    m_driver.writeAckBit(slaveParityOk ? AckType::ACK : AckType::NAK);
  } else {
    std::ignore = m_driver.readAckBit();
  }

  if (not slaveParityOk) {
    return std::nullopt;
  }

  // CONTROL
  auto const optionalControlData = readVerifiedField(CONTROL_BIT_SIZE, isTargeted);
  if (not optionalControlData) {
    return std::nullopt;
  }
  message.control = static_cast<Byte>(*optionalControlData);

  // LENGTH
  auto const optionalLengthData = readVerifiedField(LENGTH_BIT_SIZE, isTargeted);
  if (not optionalLengthData) {
    return std::nullopt;
  }
  message.length = (*optionalLengthData == 0) ? 256 : static_cast<Size>(*optionalLengthData);

  // DATA
  for (Size i = 0; i < message.length; ++i) {
    auto const optionalByteData = readVerifiedField(DATA_BIT_SIZE, isTargeted);
    if (not optionalByteData) {
      return std::nullopt;
    }
    message.data[i] = static_cast<Byte>(*optionalByteData);
  }

  return message;
}

auto Controller::writeMessage(Message const& message) const noexcept -> bool {
  if (not m_driver.isEnabled()) {
    return false;
  }

  while (not m_driver.isBusFree()) {
  }

  m_driver.writeStartBit();
  //  auto const isStarted = m_driver.writeStartBit();
  //  if (not isStarted) {
  //    return std::unexpected(ErrorType::START_BIT_ARBITRATION_LOST);
  //  }

  m_driver.writeBit(static_cast<Bit>(message.broadcast));

  {
    m_driver.writeBits(message.master, ADDRESS_BIT_SIZE);

    auto const parityBit = calculateParity(message.master, ADDRESS_BIT_SIZE);
    m_driver.writeBit(parityBit);
  }

  {
    m_driver.writeBits(message.slave, ADDRESS_BIT_SIZE);

    auto const parityBit = calculateParity(message.slave, ADDRESS_BIT_SIZE);
    m_driver.writeBit(parityBit);

    if (message.broadcast == BroadcastType::FOR_DEVICE) {
      auto const optionalAckBit = m_driver.readAckBit();
      if (not optionalAckBit) {
        return false;
      }

      auto const ackBit = optionalAckBit.value();
      if (ackBit == AckType::NAK) {
        return false;
      }

    } else {
      m_driver.writeAckBit(AckType::NAK);
    }
  }

  {
    m_driver.writeBits(message.control, CONTROL_BIT_SIZE);

    auto const parityBit = calculateParity(message.control, CONTROL_BIT_SIZE);
    m_driver.writeBit(parityBit);

    if (message.broadcast == BroadcastType::FOR_DEVICE) {
      auto const optionalAckBit = m_driver.readAckBit();
      if (not optionalAckBit) {
        return false;
      }

      auto const ackBit = optionalAckBit.value();
      if (ackBit == AckType::NAK) {
        return false;
      }

    } else {
      m_driver.writeAckBit(AckType::NAK);
    }
  }

  {
    m_driver.writeBits(message.length, LENGTH_BIT_SIZE);

    auto const parityBit = calculateParity(message.length, LENGTH_BIT_SIZE);
    m_driver.writeBit(parityBit);

    if (message.broadcast == BroadcastType::FOR_DEVICE) {
      auto const optionalAckBit = m_driver.readAckBit();
      if (not optionalAckBit) {
        return false;
      }

      auto const ackBit = optionalAckBit.value();
      if (ackBit == AckType::NAK) {
        return false;
      }

    } else {
      m_driver.writeAckBit(AckType::NAK);
    }
  }

  for (Size i = 0; i < message.length; i++) {
    m_driver.writeBits(message.data[i], DATA_BIT_SIZE);

    auto const parityBit = calculateParity(message.data[i], DATA_BIT_SIZE);
    m_driver.writeBit(parityBit);

    if (message.broadcast == BroadcastType::FOR_DEVICE) {
      auto const optionalAckBit = m_driver.readAckBit();
      if (not optionalAckBit) {
        return false;
      }

      auto const ackBit = optionalAckBit.value();
      if (ackBit == AckType::NAK) {
        return false;
      }

    } else {
      m_driver.writeAckBit(AckType::NAK);
    }
  }

  return true;
}

auto Controller::readVerifiedField(Size const bitSize, bool const sendAck) const noexcept -> std::optional<Data> {
  auto const optionalData   = m_driver.readBits(bitSize);
  auto const optionalParity = m_driver.readBit();

  if (not optionalData or not optionalParity) {
    return std::nullopt;
  }

  auto const data    = optionalData.value();
  auto const parity  = optionalParity.value();
  auto const isValid = checkParity(data, bitSize, parity);

  if (sendAck) {
    m_driver.writeAckBit(isValid ? AckType::ACK : AckType::NAK);
  } else {
    std::ignore = m_driver.readAckBit();
  }

  if (not isValid) {
    return std::nullopt;
  }

  return data;
}

} // namespace iebus
