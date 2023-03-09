// Copyright 2023 eSOL Co.,Ltd.
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

#ifndef RCLCPP__THREADS__STD__THREAD_HPP_
#define RCLCPP__THREADS__STD__THREAD_HPP_

#include <unistd.h>
#include <thread>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <utility>

#include "rclcpp/threads/std/thread_attribute.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

struct Thread
{
  using NativeHandleType = std::thread::native_handle_type;
  using Attribute = detail::ThreadAttribute;
  using Id = std::thread::id;

  Thread() noexcept
  : thread_{} {}
  Thread(Thread &&) = default;
  template<typename F, typename ... Args,
    typename = std::enable_if_t<!std::is_same<std::decay_t<F>, Attribute>::value>>
  explicit Thread(F && f, Args && ... args)
  : thread_(std::forward<F>(f), std::forward<Args>(args)...)
  {}
  template<typename F, typename ... Args>
  Thread(Attribute & attr, F && f, Args && ... args)
  {
    if (attr.set_value) {
      throw std::runtime_error("std::thread can't set thread attribute");
    }
    thread_ = std::thread(std::forward<F>(f), std::forward<Args>(args)...);
  }
  Thread(Thread const &) = delete;
  ~Thread() {}

  Thread & operator=(Thread && other) noexcept = default;
  Thread & operator=(Thread const &) = delete;

  void swap(Thread & other)
  {
    using std::swap;
    swap(thread_, other.thread_);
  }

  void join()
  {
    thread_.join();
  }

  bool joinable() const noexcept
  {
    return thread_.joinable();
  }

  void detach()
  {
    thread_.detach();
  }

  NativeHandleType native_handle()
  {
    return thread_.native_handle();
  }

  Id get_id() const noexcept
  {
    return thread_.get_id();
  }

  static int hardware_concurrency() noexcept
  {
    return std::thread::hardware_concurrency();
  }

private:
  std::thread thread_;
};

inline void swap(Thread & t1, Thread & t2)
{
  t1.swap(t2);
}

namespace this_thread
{

using std::this_thread::get_id;

}  // namespace this_thread

}  // namespace rclcpp

#endif  // RCLCPP__THREADS__STD__THREAD_HPP_
