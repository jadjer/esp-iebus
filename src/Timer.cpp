//
// Created by jadjer on 27.01.2026.
//

#include "iebus/Timer.hpp"

namespace iebus {

namespace {

Timer::Time constexpr TIMER_RESOLUTION_HZ     = 40'000'000;
Timer::Time constexpr MICROSECONDS_PER_SECOND = 1'000'000;

auto constexpr convertCountsToTimeUS(Timer::Time const counts) -> Timer::Time {
  return ((counts * MICROSECONDS_PER_SECOND) / TIMER_RESOLUTION_HZ);
}

} // namespace

Timer::Timer() noexcept {
  gptimer_config_t const timerConfiguration = {
      .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
      .direction     = GPTIMER_COUNT_UP,
      .resolution_hz = TIMER_RESOLUTION_HZ,
      .intr_priority = 0,
      .flags         = {.intr_shared = false, .allow_pd = false},
  };

  ESP_ERROR_CHECK(gptimer_new_timer(&timerConfiguration, &m_timerHandle));
  ESP_ERROR_CHECK(gptimer_enable(m_timerHandle));
  ESP_ERROR_CHECK(gptimer_start(m_timerHandle));
}

Timer::~Timer() noexcept {
  ESP_ERROR_CHECK(gptimer_stop(m_timerHandle));
  ESP_ERROR_CHECK(gptimer_disable(m_timerHandle));
  ESP_ERROR_CHECK(gptimer_del_timer(m_timerHandle));
}

auto Timer::reset() const noexcept -> void {
  ESP_ERROR_CHECK(gptimer_set_raw_count(m_timerHandle, 0));
}

auto Timer::getTimeUS() const noexcept -> Timer::Time {
  Timer::Time counts = 0;

  ESP_ERROR_CHECK(gptimer_get_raw_count(m_timerHandle, &counts));

  return convertCountsToTimeUS(counts);
}

} // namespace iebus
