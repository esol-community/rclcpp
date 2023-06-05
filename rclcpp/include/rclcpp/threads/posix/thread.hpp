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

#ifndef RCLCPP__THREADS__POSIX__THREAD_HPP_
#define RCLCPP__THREADS__POSIX__THREAD_HPP_

#include <pthread.h>
#include <unistd.h>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <utility>

#include "rclcpp/threads/posix/thread_attribute.hpp"
#include "rclcpp/threads/posix/thread_func.hpp"
#include "rclcpp/threads/posix/thread_id.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

struct Thread
{
  using NativeHandleType = pthread_t;
  using Attribute = detail::ThreadAttribute;
  using Id = detail::ThreadId;

  // Assume pthread_t is an invalid handle if it's 0
  Thread() noexcept
  : handle_{} {}
  Thread(Thread && other)
  : handle_(other.handle_) {other.handle_ = NativeHandleType{};}
  template<typename F, typename ... Args,
    typename = std::enable_if_t<!std::is_same<std::decay_t<F>, Attribute>::value>>
  explicit Thread(F && f, Args && ... args)
  : Thread(static_cast<Attribute *>(nullptr),
      make_thread_func(nullptr, std::forward<F>(f), std::forward<Args>(args)...))
  {}
  template<typename F, typename ... Args>
  Thread(Attribute & attr, F && f, Args && ... args)
  : Thread(&attr, make_thread_func(&attr, std::forward<F>(f), std::forward<Args>(args)...))
  {}
  Thread(Thread const &) = delete;
  ~Thread()
  {
    if (handle_) {std::terminate();}
  }

  Thread & operator=(Thread && other) noexcept
  {
    if (handle_) {std::terminate();}
    swap(other);
    return *this;
  }

  Thread & operator=(Thread const &) = delete;

  void swap(Thread & other)
  {
    using std::swap;
    swap(handle_, other.handle_);
  }

  void join()
  {
    void * p;
    int r = pthread_join(handle_, &p);
    if (r != 0) {throw std::system_error(r, std::system_category(), "Error in pthread_join ");}
    handle_ = NativeHandleType{};
  }

  bool joinable() const noexcept
  {
    return 0 == pthread_equal(handle_, NativeHandleType{});
  }

  void detach()
  {
    int r = pthread_detach(handle_);
    if (r != 0) {throw std::system_error(r, std::system_category(), "Error in detach ");}
    handle_ = NativeHandleType{};
  }

  NativeHandleType native_handle() const
  {
    return handle_;
  }

  Id get_id() const noexcept
  {
    return Id{handle_};
  }

  static int hardware_concurrency() noexcept
  {
    return static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
  }

private:
  using ThreadFuncUniquePtr = detail::ThreadFuncUniquePtr;

  Thread(Attribute * attr, ThreadFuncUniquePtr func);

  template<typename F, typename ... Args>
  static ThreadFuncUniquePtr make_thread_func(Attribute * attr, F && f, Args && ... args)
  {
    using detail::ThreadFunc;
    using detail::ThreadFuncImplementation;
    ThreadFunc * func = new ThreadFuncImplementation<std::decay_t<F>, std::decay_t<Args>...>(
      attr, std::forward<F>(f), std::forward<Args>(args)...
    );

    return ThreadFuncUniquePtr{func};
  }

  NativeHandleType handle_;
};

inline void swap(Thread & t1, Thread & t2)
{
  t1.swap(t2);
}

}  // namespace rclcpp

#endif  // RCLCPP__THREADS__POSIX__THREAD_HPP_
