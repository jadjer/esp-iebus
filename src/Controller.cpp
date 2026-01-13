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
#include <utility>

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

  ESP_LOGE(TAG, "%u", static_cast<Bit>(expectedResult.error()));

  return false;
}

auto Controller::readMessage() const noexcept -> std::expected<Message, BitError> {
  if (not m_driver.isEnabled()) {
    BitResult const bitResult = {
        .pulseWidth = 0,
        .bitType    = BitType::BIT_UNKNOWN,
    };

    BitError const bitError = {
        .bitResult = bitResult,
        .errorType  = ErrorType::CONTROLLER_DISABLED,
    };

    return std::unexpected(bitError);
  }

  /**
   * START BIT
   */
  {
    auto const expectedData = m_driver.readStartBit();
    if (not expectedData) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::START_BIT_READ_ERROR,
      };

      return std::unexpected(bitError);
    }
  }

  Message message = {};

  /**
   * BROADCAST
   */
  {
    auto const expectedData = m_driver.readBit();
    if (not expectedData) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::BROADCAST_BIT_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const data = expectedData.value();

    message.broadcast = static_cast<BroadcastType>(data);
  }

  /**
   * MASTER
   */
  {
    auto const expectedData = m_driver.readBits(ADDRESS_BIT_SIZE);
    if (not expectedData) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::MASTER_ADDRESS_DATA_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const expectedParityBit = m_driver.readBit();
    if (not expectedParityBit) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::MASTER_ADDRESS_PARITY_BIT_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const data      = expectedData.value();
    auto const parityBit = expectedParityBit.value();

    message.master = static_cast<Address>(data);

    auto const isParityValid = checkParity(data, ADDRESS_BIT_SIZE, parityBit);
    if (not isParityValid) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::MASTER_ADDRESS_PARITY_WRONG,
      };

      return std::unexpected(bitError);
    }
  }

  /**
   * SLAVE
   */
  {
    auto const expectedData = m_driver.readBits(ADDRESS_BIT_SIZE);
    if (not expectedData) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::SLAVE_ADDRESS_DATA_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const expectedParityBit = m_driver.readBit();
    if (not expectedParityBit) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::SLAVE_ADDRESS_PARITY_BIT_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const data      = expectedData.value();
    auto const parityBit = expectedParityBit.value();

    message.slave = static_cast<Address>(data);

    auto const isPersonal     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isTargeted     = message.slave == m_address;
    auto const isAnswerNeeded = isPersonal and isTargeted;

    if (isAnswerNeeded) {
      auto const isParityValid = checkParity(data, ADDRESS_BIT_SIZE, parityBit);

      m_driver.writeAckBit(isParityValid ? AckType::ACK : AckType::NAK);

      if (not isParityValid) {
        auto const bitResult = expectedData.error();

        BitError const bitError = {
            .bitResult = bitResult,
            .errorType  = ErrorType::SLAVE_ADDRESS_PARITY_WRONG,
        };

        return std::unexpected(bitError);
      }

    } else {
      std::ignore = m_driver.readAckBit();
    }
  }

  /**
   * CONTROL
   */
  {
    auto const expectedData = m_driver.readBits(CONTROL_BIT_SIZE);
    if (not expectedData) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::CONTROL_DATA_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const expectedParityBit = m_driver.readBit();
    if (not expectedParityBit) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::CONTROL_PARITY_BIT_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const data      = expectedData.value();
    auto const parityBit = expectedParityBit.value();

    message.control = static_cast<Byte>(data);

    auto const isPersonal     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isTargeted     = message.slave == m_address;
    auto const isAnswerNeeded = isPersonal and isTargeted;

    if (isAnswerNeeded) {
      auto const isParityValid = checkParity(data, CONTROL_BIT_SIZE, parityBit);

      m_driver.writeAckBit(isParityValid ? AckType::ACK : AckType::NAK);

      if (not isParityValid) {
        auto const bitResult = expectedData.error();

        BitError const bitError = {
            .bitResult = bitResult,
            .errorType  = ErrorType::CONTROL_PARITY_WRONG,
        };

        return std::unexpected(bitError);
      }

    } else {
      std::ignore = m_driver.readAckBit();
    }
  }

  /**
   * LENGTH
   */
  {
    auto const expectedData = m_driver.readBits(LENGTH_BIT_SIZE);
    if (not expectedData) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::LENGTH_DATA_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const expectedParityBit = m_driver.readBit();
    if (not expectedParityBit) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::LENGTH_PARITY_BIT_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const data      = expectedData.value();
    auto const parityBit = expectedParityBit.value();

    if (data == 0) {
      message.length = static_cast<Size>(256);
    } else {
      message.length = static_cast<Size>(data);
    }

    auto const isPersonal     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isTargeted     = message.slave == m_address;
    auto const isAnswerNeeded = isPersonal and isTargeted;

    if (isAnswerNeeded) {
      auto const isParityValid = checkParity(data, LENGTH_BIT_SIZE, parityBit);

      m_driver.writeAckBit(isParityValid ? AckType::ACK : AckType::NAK);

      if (not isParityValid) {
        auto const bitResult = expectedData.error();

        BitError const bitError = {
            .bitResult = bitResult,
            .errorType  = ErrorType::LENGTH_PARITY_WRONG,
        };

        return std::unexpected(bitError);
      }

    } else {
      std::ignore = m_driver.readAckBit();
    }
  }

  /**
   * DATA
   */
  for (Size i = 0; i < message.length; i++) {
    auto const expectedData = m_driver.readBits(DATA_BIT_SIZE);
    if (not expectedData) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::DATA_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const expectedParityBit = m_driver.readBit();
    if (not expectedParityBit) {
      auto const bitResult = expectedData.error();

      BitError const bitError = {
          .bitResult = bitResult,
          .errorType  = ErrorType::DATA_PARITY_BIT_READ_ERROR,
      };

      return std::unexpected(bitError);
    }

    auto const data      = expectedData.value();
    auto const parityBit = expectedParityBit.value();

    message.data[i] = static_cast<Byte>(data);

    auto const isPersonal     = message.broadcast == BroadcastType::FOR_DEVICE;
    auto const isTargeted     = message.slave == m_address;
    auto const isAnswerNeeded = isPersonal and isTargeted;

    if (isAnswerNeeded) {
      auto const isParityValid = checkParity(data, DATA_BIT_SIZE, parityBit);

      m_driver.writeAckBit(isParityValid ? AckType::ACK : AckType::NAK);

      if (not isParityValid) {
        auto const bitResult = expectedData.error();

        BitError const bitError = {
            .bitResult = bitResult,
            .errorType  = ErrorType::DATA_PARITY_WRONG,
        };

        return std::unexpected(bitError);
      }

    } else {
      std::ignore = m_driver.readAckBit();
    }
  }

  return message;
}

auto Controller::writeMessage(Message const& message) const noexcept -> std::expected<bool, ErrorType> {
  if (not m_driver.isEnabled()) {
    return std::unexpected(ErrorType::CONTROLLER_DISABLED);
  }

  while (not m_driver.isBusFree()) {
  }

  auto const isStarted = m_driver.writeStartBit();
  if (not isStarted) {
    return std::unexpected(ErrorType::START_BIT_ARBITRATION_LOST);
  }

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
      auto const expectedAckBit = m_driver.readAckBit();
      if (not expectedAckBit) {
        return std::unexpected(ErrorType::SLAVE_ADDRESS_ACK_BIT_READ_ERROR);
      }

      auto const ackBit = expectedAckBit.value();
      if (ackBit == AckType::NAK) {
        return std::unexpected(ErrorType::SLAVE_ADDRESS_ACK_WRONG);
      }
    } else {
      m_driver.writeAckBit(AckType::ACK);
    }
  }

  {
    m_driver.writeBits(message.control, CONTROL_BIT_SIZE);

    auto const parityBit = calculateParity(message.control, CONTROL_BIT_SIZE);
    m_driver.writeBit(parityBit);

    if (message.broadcast == BroadcastType::FOR_DEVICE) {
      auto const expectedAckBit = m_driver.readAckBit();
      if (not expectedAckBit) {
        return std::unexpected(ErrorType::CONTROL_ACK_BIT_READ_ERROR);
      }

      auto const ackBit = expectedAckBit.value();
      if (ackBit == AckType::NAK) {
        return std::unexpected(ErrorType::CONTROL_ACK_WRONG);
      }
    } else {
      m_driver.writeAckBit(AckType::ACK);
    }
  }

  {
    m_driver.writeBits(message.length, LENGTH_BIT_SIZE);

    auto const parityBit = calculateParity(message.length, LENGTH_BIT_SIZE);
    m_driver.writeBit(parityBit);

    if (message.broadcast == BroadcastType::FOR_DEVICE) {
      auto const expectedAckBit = m_driver.readAckBit();
      if (not expectedAckBit) {
        return std::unexpected(ErrorType::LENGTH_ACK_BIT_READ_ERROR);
      }

      auto const ackBit = expectedAckBit.value();
      if (ackBit == AckType::NAK) {
        return std::unexpected(ErrorType::LENGTH_ACK_WRONG);
      }
    } else {
      m_driver.writeAckBit(AckType::ACK);
    }
  }

  for (Size i = 0; i < message.length; i++) {
    m_driver.writeBits(message.data[i], DATA_BIT_SIZE);

    auto const parityBit = calculateParity(message.data[i], DATA_BIT_SIZE);
    m_driver.writeBit(parityBit);

    if (message.broadcast == BroadcastType::FOR_DEVICE) {
      auto const expectedAckBit = m_driver.readAckBit();
      if (not expectedAckBit) {
        return std::unexpected(ErrorType::DATA_ACK_BIT_READ_ERROR);
      }

      auto const ackBit = expectedAckBit.value();
      if (ackBit == AckType::NAK) {
        return std::unexpected(ErrorType::DATA_ACK_WRONG);
      }
    } else {
      m_driver.writeAckBit(AckType::ACK);
    }
  }

  return true;
}

} // namespace iebus
