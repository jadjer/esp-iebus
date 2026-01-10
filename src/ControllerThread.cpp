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

#include "iebus/ControllerThread.hpp"

namespace iebus {

ControllerThread::ControllerThread(Pin rx, Pin tx, Pin enable, Address address) noexcept : Controller(rx, tx, enable, address) {
}

auto ControllerThread::start() -> void {
  if (m_threadEnable) {
    return;
  }

  m_threadEnable = true;

  m_thread = Thread(&ControllerThread::process, this);
}

auto ControllerThread::stop() -> void {
  {
    std::lock_guard lock(m_mutex);
    m_threadEnable = false;
  }

  m_cv.notify_all();

  if (m_thread.joinable()) {
    m_thread.join();
  }
}

auto ControllerThread::isStarted() const -> bool {
  return m_threadEnable;
}

auto ControllerThread::hasMessage() -> bool {
  std::lock_guard lockGuard(m_mutex);

  return not m_queue.empty();
}

auto ControllerThread::getMessage() -> std::optional<Message> {
  std::unique_lock lock(m_mutex);

  m_cv.wait(lock, [this] {
    return not m_queue.empty() or not m_threadEnable;
  });

  if (m_queue.empty()) {
    return std::nullopt;
  }

  auto const message = m_queue.front();

  m_queue.pop();

  return message;
}

auto ControllerThread::process() -> void {
  while (m_threadEnable) {
    auto const expectedMessage = readMessage();

    if (expectedMessage.has_value()) {
      auto const message = expectedMessage.value();

      {
        std::lock_guard lock(m_mutex);
        m_queue.push(message);
      }

      m_cv.notify_one();
    }
  }
}

} // namespace iebus
