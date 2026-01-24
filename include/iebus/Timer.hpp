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

#include <iebus/types.hpp>
#include <driver/gptimer_types.h>

namespace iebus {

class Timer {
private:
  using TimerHandle = gptimer_handle_t;

public:
  Timer() noexcept;
  ~Timer() noexcept;

public:
  auto start() const noexcept -> void;
  auto stop() const noexcept -> void;

public:
  auto reset() const noexcept -> void;

public:
  auto getTime() const noexcept -> Time;

private:
  TimerHandle m_timerHandle = nullptr;
};

}
