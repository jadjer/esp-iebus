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

#pragma once

#include <optional>

#include <iebus/Driver.hpp>
#include <iebus/Message.hpp>

namespace iebus {

/**
 * @class Controller
 * IEBus Controller
 */
class Controller {

public:
    Controller(Driver::Pin rx, Driver::Pin tx, Driver::Pin enable, Address address) noexcept;

public:
    /**
     * Enable IEBus driver
     */
    auto enable() -> void;
    /**
     * Enable IEBus driver
     */
    auto disable() -> void;

public:
    /**
     * Check if IEBus controller enabled
     * @return bool
     */
    [[nodiscard]] auto isEnabled() const -> bool;

public:
    /**
     * Read the message from IEBus
     * @return Optional message
     */
    [[nodiscard]] auto readMessage() const -> std::optional<Message>;
    /**
     * Write a message to IEBus
     * @param message Message
     * @return bool
     */
    [[nodiscard]] auto writeMessage(Message const& message) const -> bool;

private:
    /**
     * Check parity for calculated parity
     * @param data Data for check
     * @param size Data size
     * @param parity Etalon parity
     * @return Comparison result
     */
    static auto checkParity(Data data, Size size, Bit parity) -> bool;
    /**
     * Calculate parity for calculated parity
     * @param data Data for calculate
     * @param size Data size
     * @return Parity bit
     */
    static auto calculateParity(Data data, Size size) -> Bit;

private:
    Address const m_address;

private:
    Driver m_driver;
};

} // namespace iebus
