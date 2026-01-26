//
// Created by jadjer on 27.01.2026.
//

#pragma once

#include <cstdint>
#include <driver/gptimer.h>

/**
 * @namespace iebus
 */
namespace iebus {

/**
 * @class TImer
 */
class Timer {
public:
  using Time = std::uint64_t;

private:
  using TimerHandle = gptimer_handle_t;

public:
  Timer() noexcept;
  ~Timer() noexcept;

public:
  /**
   * Get current time
   * @return Time
   */
  [[nodiscard]] auto getTimeUS() const noexcept -> Time;

public:
  /**
   * Reset timer
   */
  auto reset() const noexcept -> void;

private:
  TimerHandle m_timerHandle = nullptr;
};

} // namespace iebus
