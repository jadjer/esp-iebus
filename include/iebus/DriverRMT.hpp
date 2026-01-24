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
// Created by jadjer on 24.01.2026.
//

#pragma once

#include <driver/rmt_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <iebus/types.hpp>
#include <optional>

namespace iebus {

class DriverRMT {
private:
  using FrameWidth = std::uint32_t;

public:
  DriverRMT(Pin rx, Pin tx, Pin enable) noexcept;
  ~DriverRMT() noexcept;

public:
  [[nodiscard]] auto isEnabled() const noexcept -> bool;
  [[nodiscard]] auto isBusHigh() const noexcept -> bool;
  [[nodiscard]] auto isBusLow() const noexcept -> bool;
  [[nodiscard]] auto isBusFree() const noexcept -> bool;

public:
  auto enable() noexcept -> void;
  auto disable() noexcept -> void;

public:
  auto readStartBit() noexcept -> bool;
  auto readBits(Size numBits) noexcept -> std::optional<Data>;

private:
  auto readPulseWidth(FrameWidth timeout) noexcept -> std::optional<Time>;

public:
  auto writeStartBit() noexcept -> void;
  auto writeBits(Data data, Size numBits) noexcept -> void;

private:
  auto writePulseWidth(Time pulse, Time frame) noexcept -> void;

private:
  Pin const m_rxPin;
  Pin const m_txPin;
  Pin const m_enablePin;

private:
  bool m_enable                         = false;
  QueueHandle_t m_queueHandler          = nullptr;
  rmt_channel_handle_t m_receiverChannel = nullptr;
  rmt_channel_handle_t m_transceiverChannel = nullptr;
};

} // namespace iebus
