//
// Created by jadjer on 24.01.2026.
//

#include "iebus/Timer.hpp"

#include <driver/gptimer.h>

namespace iebus {

namespace {

auto constexpr TIMER_RESOLUTION_HZ = 1'000'000;

}

Timer::Timer() noexcept {
  gptimer_config_t const timerConfiguration = {
      .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
      .direction     = GPTIMER_COUNT_UP,
      .resolution_hz = TIMER_RESOLUTION_HZ,
      .intr_priority = 0,
      .flags         = {},
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timerConfiguration, &m_timerHandle));

  ESP_ERROR_CHECK(gptimer_enable(m_timerHandle));
}

Timer::~Timer() noexcept {
  ESP_ERROR_CHECK(gptimer_stop(m_timerHandle));
  ESP_ERROR_CHECK(gptimer_disable(m_timerHandle));
  ESP_ERROR_CHECK(gptimer_del_timer(m_timerHandle));
}

auto Timer::start() const noexcept -> void {
  ESP_ERROR_CHECK(gptimer_start(m_timerHandle));
}

auto Timer::stop() const noexcept -> void {
  ESP_ERROR_CHECK(gptimer_stop(m_timerHandle));
}

auto Timer::reset() const noexcept -> void {
  ESP_ERROR_CHECK(gptimer_set_raw_count(m_timerHandle, 0));
}

auto Timer::getTime() const noexcept -> Time {
  Time time = 0;

  ESP_ERROR_CHECK(gptimer_get_raw_count(m_timerHandle, &time));

  return time;
}

} // namespace iebus
