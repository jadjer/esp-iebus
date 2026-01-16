//
// Created by jadjer on 14.01.2026.
//

#include "iebus/Timer.hpp"

namespace iebus {

namespace {

auto constexpr DEFAULT_RESOLUTION_HZ = 1 * 1000 * 1000;

} // namespace

Timer::Timer() : m_timer(nullptr) {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

  gptimer_config_t const timerConfig = {
      .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
      .direction     = GPTIMER_COUNT_UP,
      .resolution_hz = DEFAULT_RESOLUTION_HZ,
  };

#pragma GCC diagnostic pop

  ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &m_timer));
}

Timer::~Timer() {
  if (not m_timer) {
    return;
  }

  ESP_ERROR_CHECK(gptimer_del_timer(m_timer));
}

auto Timer::enable() -> void {
  if (not m_timer) {
    return;
  }

  ESP_ERROR_CHECK(gptimer_enable(m_timer));
}

auto Timer::disable() -> void {
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
