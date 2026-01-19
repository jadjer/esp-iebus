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

#include "iebus/Timer.hpp"

namespace iebus {

namespace {

auto constexpr DEFAULT_RESOLUTION_HZ = 1 * 1000 * 1000;

} // namespace

Timer::Timer() : m_timer(nullptr) {
  gptimer_config_t const timerConfig = {
      .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
      .direction     = GPTIMER_COUNT_UP,
      .resolution_hz = DEFAULT_RESOLUTION_HZ,
      .intr_priority = 0,
      .flags         = {},
  };

  ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &m_timer));
}

Timer::~Timer() {
  if (not m_timer) {
    return;
  }

  ESP_ERROR_CHECK(gptimer_del_timer(m_timer));
}

auto Timer::enable() const -> void {
  if (not m_timer) {
    return;
  }

  ESP_ERROR_CHECK(gptimer_enable(m_timer));
}

auto Timer::disable() const -> void {
  if (not m_timer) {
    return;
  }

  ESP_ERROR_CHECK(gptimer_disable(m_timer));
}

auto Timer::start() const -> void {
  if (not m_timer) {
    return;
  }

  ESP_ERROR_CHECK(gptimer_start(m_timer));
}

auto Timer::stop() const -> void {
  if (not m_timer) {
    return;
  }

  ESP_ERROR_CHECK(gptimer_stop(m_timer));
}

auto Timer::reset() const -> void {
  if (not m_timer) {
    return;
  }

  ESP_ERROR_CHECK(gptimer_set_raw_count(m_timer, 0));
}

auto Timer::getTime() const -> Time {
  if (not m_timer) {
    return 0;
  }

  Time currentTime = 0;

  ESP_ERROR_CHECK(gptimer_get_raw_count(m_timer, &currentTime));

  return currentTime;
}

} // namespace iebus