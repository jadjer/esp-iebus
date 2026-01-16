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
  auto enable() -> void;
  /**
   * Disable timer
   */
  auto disable() -> void;

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
