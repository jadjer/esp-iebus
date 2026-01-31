//
// Created by jadjer on 27.01.2026.
//

#include "iebus/Timer.hpp"

#include <esp_attr.h>
#include <esp_timer.h>

namespace iebus {

auto IRAM_ATTR Timer::getTime() const noexcept -> Timer::Time {
  return (esp_timer_get_time() - m_startTime);
}

auto IRAM_ATTR Timer::reset() noexcept -> void {
  m_startTime = esp_timer_get_time();
}

} // namespace iebus
