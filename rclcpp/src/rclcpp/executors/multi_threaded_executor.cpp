// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

using rclcpp::executors::MultiThreadedExecutor;

MultiThreadedExecutor::MultiThreadedExecutor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: rclcpp::Executor(options),
  yield_before_execute_(yield_before_execute),
  next_exec_timeout_(next_exec_timeout),
  use_thread_attrs_(false)
{
  number_of_threads_ = number_of_threads > 0 ?
    number_of_threads :
    std::max(std::thread::hardware_concurrency(), 2U);

  if (number_of_threads_ == 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "MultiThreadedExecutor is used with a single thread.\n"
      "Use the SingleThreadedExecutor instead.");
  }

  thread_attributes_ = nullptr;
  if (RCL_RET_OK ==
    rcl_arguments_get_thread_attrs(
      &options.context->get_rcl_context()->global_arguments,
      &thread_attributes_))
  {
    if (nullptr != thread_attributes_) {
      if (0 != thread_attributes_->num_attributes) {
        use_thread_attrs_ = true;
      } else {
        thread_attributes_ = nullptr;
        if (RCL_RET_OK ==
          rcl_context_get_thread_attrs(
            options.context->get_rcl_context().get(),
            &thread_attributes_))
        {
          if (nullptr != thread_attributes_) {
            if (0 != thread_attributes_->num_attributes) {
              use_thread_attrs_ = true;
            }
          }
        }
      }
    }
  }
  if (true == use_thread_attrs_ &&
    0 != number_of_threads &&
    thread_attributes_->num_attributes != number_of_threads)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "The number of thread attribute and number_of_threads is different.\n"
      "MultiThreadedExecutor is used with the number of threads specified by thread attribute.");
  }
}

MultiThreadedExecutor::~MultiThreadedExecutor() {}

void
MultiThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  std::vector<rclcpp::Thread> threads;
  size_t thread_id = 0;

  if (use_thread_attrs_) {
    rclcpp::detail::ThreadAttribute thread_attr;
    {
      std::lock_guard wait_lock{wait_mutex_};
      for (; thread_id < thread_attributes_->num_attributes - 1; ++thread_id) {
        thread_attr.set_thread_attributes(
          thread_attributes_->attributes[thread_id]);
        auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
        threads.emplace_back(rclcpp::Thread(thread_attr, func));
      }
      thread_attr.set_thread_attributes(
        thread_attributes_->attributes[thread_id]);
      run_on_rclcpp_thread(thread_attr, threads, thread_id);
    }
    run_on_this_thread(thread_attr, thread_id);
  } else {
    {
      std::lock_guard wait_lock{wait_mutex_};
      for (; thread_id < number_of_threads_ - 1; ++thread_id) {
        auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
        threads.emplace_back(func);
      }
    }
    run(thread_id);
  }

  for (auto & thread : threads) {
    thread.join();
  }
}

size_t
MultiThreadedExecutor::get_number_of_threads()
{
  return number_of_threads_;
}

void
MultiThreadedExecutor::run(size_t this_thread_number)
{
  (void)this_thread_number;
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_exec;
    {
      std::lock_guard wait_lock{wait_mutex_};
      if (!rclcpp::ok(this->context_) || !spinning.load()) {
        return;
      }
      if (!get_next_executable(any_exec, next_exec_timeout_)) {
        continue;
      }
    }
    if (yield_before_execute_) {
      std::this_thread::yield();
    }

    execute_any_executable(any_exec);

    // Clear the callback_group to prevent the AnyExecutable destructor from
    // resetting the callback group `can_be_taken_from`
    any_exec.callback_group.reset();
  }
}

#if defined(__linux__)
#include <pthread.h>
void
MultiThreadedExecutor::run_on_rclcpp_thread(
  rclcpp::detail::ThreadAttribute & thread_attr,
  std::vector<rclcpp::Thread> & threads,
  size_t thread_id)
{
  (void)thread_attr;
  (void)threads;
  (void)thread_id;
}

void
MultiThreadedExecutor::run_on_this_thread(
  rclcpp::detail::ThreadAttribute & thread_attr,
  size_t thread_id)
{
  int r;
  r = sched_setaffinity(0, sizeof(cpu_set_t), thread_attr.get_affinity().native_cpu_set());
  if (r != 0) {
    throw std::system_error(r, std::system_category(), "Error in sched_setaffinity ");
  }
  sched_param param;
  param.sched_priority = thread_attr.get_priority();
  int policy = thread_attr.get_sched_policy();
  r = sched_setscheduler(
    0, policy,
    &param);
  if (r != 0) {
    throw std::system_error(r, std::system_category(), "Error in sched_setscheduler ");
  }
  run(thread_id);
}

#elif defined(_WIN32)
#include <windows.h>
void
MultiThreadedExecutor::run_on_rclcpp_thread(
  rclcpp::detail::ThreadAttribute & thread_attr,
  std::vector<rclcpp::Thread> & threads,
  size_t thread_id)
{
  (void)thread_attr;
  (void)threads;
  (void)thread_id;
}

void
MultiThreadedExecutor::run_on_this_thread(
  rclcpp::detail::ThreadAttribute & thread_attr,
  size_t thread_id)
{
  // TO DO: Setting parent thread attribute on Windows
  run(thread_id);
}
#else
void
MultiThreadedExecutor::run_on_rclcpp_thread(
  rclcpp::detail::ThreadAttribute & thread_attr,
  std::vector<rclcpp::Thread> & threads,
  size_t thread_id)
{
  auto func = std::bind(&MultiThreadedExecutor::run, this, thread_id);
  threads.emplace_back(rclcpp::Thread(thread_attr, func));
}

void
MultiThreadedExecutor::run_on_this_thread(
  rclcpp::detail::ThreadAttribute & thread_attr,
  size_t thread_id)
{
  (void)thread_attr;
  (void)thread_id;
}

#endif
