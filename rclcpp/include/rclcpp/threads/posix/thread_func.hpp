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

#ifndef RCLCPP__THREADS__POSIX__THREAD_FUNC_HPP_
#define RCLCPP__THREADS__POSIX__THREAD_FUNC_HPP_

#include <cstddef>
#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>

#include "rclcpp/threads/posix/thread_attribute.hpp"

namespace rclcpp::detail
{

struct ThreadFunc
{
  virtual ~ThreadFunc() = default;
  virtual void run() = 0;
};

template<typename F, typename ... Args>
struct ThreadFuncImplementation : ThreadFunc
{
  template<typename G, typename ... As>
  ThreadFuncImplementation(rclcpp::detail::ThreadAttribute * attr, G && g, As && ... as)
  : func_(std::forward<G>(g)), args_(std::forward<As>(as)...)
  {
    if (nullptr != attr) {
      sched_policy_ = attr->get_sched_policy();
      priority_ = attr->get_priority();
    } else {
      sched_policy_ = -1;
      priority_ = -1;
    }
  }

private:
  static constexpr bool is_memfun_ = std::is_member_function_pointer_v<F>;

  void run() override
  {
    if (sched_policy_ != -1 && sched_policy_ != SCHED_FIFO && sched_policy_ != SCHED_RR) {
      sched_param param;
      param.sched_priority = priority_;
      int r = pthread_setschedparam(
        pthread_self(),
        sched_policy_,
        &param);
      if (r != 0) {
        throw std::system_error(r, std::system_category(), "Error in pthread_setschedparam ");
      }
    }
    run_impl(std::index_sequence_for<Args...>{});
  }

  template<std::size_t ... Is,
    bool Cond = is_memfun_,
    typename = std::enable_if_t<Cond>>
  void run_impl(std::index_sequence<0, Is...>)
  {
    (arg<0>().*std::move(func_))(arg<Is>()...);
  }
  template<std::size_t ... Is,
    bool Cond = !is_memfun_,
    typename = std::enable_if_t<Cond>,
    typename = void>
  void run_impl(std::index_sequence<Is...>)
  {
    std::move(func_)(arg<Is>()...);
  }

  template<std::size_t I>
  auto && arg()
  {
    return std::move(std::get<I>(args_));
  }

  F func_;
  std::tuple<Args...> args_;
  int sched_policy_;
  int priority_;
};

using ThreadFuncUniquePtr = std::unique_ptr<ThreadFunc>;

}  // namespace rclcpp::detail

#endif  // RCLCPP__THREADS__POSIX__THREAD_FUNC_HPP_
