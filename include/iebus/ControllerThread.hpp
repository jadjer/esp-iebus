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
// Created by jadjer on 10.01.2026.
//

#pragma once

#include <atomic>
#include <condition_variable>
#include <iebus/Controller.hpp>
#include <iebus/Message.hpp>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

/**
 * @namespace iebus
 */
namespace iebus {

/**
 * @class ControllerThread
 * Thread for IEBus Controller
 */
class ControllerThread : public Controller {
private:
  using CV     = std::condition_variable;
  using Bool   = std::atomic<bool>;
  using Mutex  = std::mutex;
  using Queue  = std::queue<Message>;
  using Thread = std::thread;

public:
  ControllerThread(Pin rx, Pin tx, Pin enable, Address address) noexcept;
  ~ControllerThread() noexcept override = default;

public:
  /**
   * Start thread
   */
  auto start() -> void;
  /**
   * Stop thread
   */
  auto stop() -> void;

public:
  /**
   * Check if thread enabled
   * @return bool
   */
  [[nodiscard]] auto isStarted() const -> bool;

public:
  /**
   * Has message in queue
   * @return bool
   */
  [[nodiscard]] auto hasMessage() -> bool;

public:
  /**
   * Get message from queue
   * @return Optional message
   */
  auto getMessage() -> std::optional<Message>;

private:
  /**
   * Thread process
   */
  auto process() -> void;

private:
  CV m_cv;
  Queue m_queue;
  Thread m_thread;
  Bool m_threadEnable = false;
  mutable Mutex m_mutex;
};

} // namespace iebus
