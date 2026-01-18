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
// Created by jadjer on 14.01.2026.
//

#pragma once

#include <driver/gptimer.h>
#include <iebus/types.hpp>

/**
 * @namespace iebus
 */
namespace iebus {

/**
 * @class Timer
 */
class Timer {
private:
  using TimerHandle = gptimer_handle_t;

public:
  Timer();
  ~Timer();

public:
  /**
   * Enable timer
   */
  auto enable() const -> void;
  /**
   * Disable timer
   */
  auto disable() const -> void;

public:
  /**
   * Start timer
   */
  auto start() const -> void;
  /**
   * Stop timer
   */
  auto stop() const -> void;

public:
  /**
   * Reset timer
   */
  auto reset() const -> void;

public:
  /**
   * Get current time
   * @return Time
   */
  [[nodiscard]] auto getTime() const -> Time;

private:
  TimerHandle m_timer;
};

} // namespace iebus
