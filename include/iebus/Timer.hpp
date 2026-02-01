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
// Created by jadjer on 27.01.2026.
//

#pragma once

#include <cstdint>

/**
 * @namespace iebus
 */
namespace iebus {

/**
 * @class Timer
 */
class Timer {
public:
  using Time = std::int64_t;

public:
  /**
   * Get current time
   * @return Time in microseconds
   */
  [[nodiscard]] auto getTime() const noexcept -> Time;

public:
  /**
   * Reset timer
   */
  auto reset() noexcept -> void;

private:
  Time m_startTime = 0;
};

} // namespace iebus
