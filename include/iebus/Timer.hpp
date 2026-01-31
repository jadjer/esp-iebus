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
 * @class TImer
 */
class Timer {
public:
  using Time = std::int64_t;

public:
  /**
   * Get current time
   * @return Time
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
