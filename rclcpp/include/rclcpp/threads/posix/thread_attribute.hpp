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

#ifndef RCLCPP__THREADS__POSIX__THREAD_ATTRIBUTE_HPP_
#define RCLCPP__THREADS__POSIX__THREAD_ATTRIBUTE_HPP_

#include <pthread.h>

#include "rcl_yaml_param_parser/types.h"
#include "rclcpp/visibility_control.hpp"

#ifdef __linux__
#include "rclcpp/threads/posix/linux/cpu_set.hpp"
#endif

namespace rclcpp
{

namespace detail
{

struct ThreadAttribute
{
  ThreadAttribute()
  {
    NativeAttributeType attr_;

    int r = pthread_attr_init(&attr_);
    if (r != 0) {throw std::system_error(r, std::system_category(), "Error in pthread_attr_init ");}

    size_t cpu_size_ = CPU_ALLOC_SIZE(static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN)));
    r = pthread_attr_getaffinity_np(&attr_, cpu_size_, cpu_set_.native_cpu_set());
    if (r != 0) {
      throw std::system_error(r, std::system_category(), "Error in pthread_attr_getaffinity_np ");
    }

    r = pthread_attr_getschedpolicy(&attr_, &sched_policy_);
    if (r != 0) {
      throw std::system_error(r, std::system_category(), "Error in pthread_attr_getschedpolicy ");
    }

    r = pthread_attr_getstacksize(&attr_, &stack_size_);
    if (r != 0) {
      throw std::system_error(r, std::system_category(), "Error in pthread_attr_getstacksize ");
    }

    sched_param param;
    r = pthread_attr_getschedparam(&attr_, &param);
    if (r != 0) {
      throw std::system_error(r, std::system_category(), "Error in pthread_attr_getschedparam ");
    }
    priority_ = param.sched_priority;

    int flag;
    r = pthread_attr_getdetachstate(&attr_, &flag);
    if (r != 0) {
      throw std::system_error(r, std::system_category(), "Error in pthread_attr_getdetachstate ");
    }
    detached_flag_ = (flag == PTHREAD_CREATE_DETACHED);
    pthread_attr_destroy(&attr_);
  }
  ~ThreadAttribute()
  {
  }

  ThreadAttribute(ThreadAttribute const &) = delete;
  ThreadAttribute(ThreadAttribute &&) = delete;
  ThreadAttribute & operator=(ThreadAttribute const &) = delete;
  ThreadAttribute & operator=(ThreadAttribute &&) = delete;

  using NativeAttributeType = pthread_attr_t;

  ThreadAttribute & set_affinity(CpuSet & rclcpp_cs)
  {
    cpu_set_ = rclcpp_cs;
    return *this;
  }
  CpuSet get_affinity()
  {
    return cpu_set_;
  }

  ThreadAttribute & set_sched_policy(rcl_thread_scheduling_policy_type_t sp)
  {
    sched_policy_ = rcl_scheduling_policy_to_sched_policy(sp);
    return *this;
  }
  int get_sched_policy() const
  {
    return sched_policy_;
  }

  ThreadAttribute & set_stack_size(std::size_t sz)
  {
    stack_size_ = sz;
    return *this;
  }
  std::size_t get_stack_size() const
  {
    return stack_size_;
  }

  ThreadAttribute & set_priority(int prio)
  {
    priority_ = prio;
    return *this;
  }
  int get_priority() const
  {
    return priority_;
  }

  ThreadAttribute & set_run_as_detached(bool detach)
  {
    detached_flag_ = detach;
    return *this;
  }

  bool get_run_as_detached() const
  {
    return detached_flag_;
  }

  void
  set_thread_attributes(
    rcl_thread_attr_t attributes)
  {
    rclcpp::detail::CpuSet cpu_set;
    cpu_set.clear();
    cpu_set.set(attributes.core_affinity);
    set_affinity(cpu_set);
    set_sched_policy(attributes.scheduling_policy);
    set_priority(attributes.priority);
  }

private:
  CpuSet cpu_set_;
  int sched_policy_;
  std::size_t stack_size_;
  int priority_;
  bool detached_flag_;

  int rcl_scheduling_policy_to_sched_policy(
    rcl_thread_scheduling_policy_type_t sched_policy)
  {
    switch (sched_policy) {
#ifdef SCHED_FIFO
      case RCL_THREAD_SCHEDULING_POLICY_FIFO:
        return SCHED_FIFO;
#endif
#ifdef SCHED_RR
      case RCL_THREAD_SCHEDULING_POLICY_RR:
        return SCHED_RR;
#endif
#ifdef SCHED_OTHER
      case RCL_THREAD_SCHEDULING_POLICY_OTHER:
        return SCHED_OTHER;
#endif
#ifdef SCHED_IDLE
      case RCL_THREAD_SCHEDULING_POLICY_IDLE:
        return SCHED_IDLE;
#endif
#ifdef SCHED_BATCH
      case RCL_THREAD_SCHEDULING_POLICY_BATCH:
        return SCHED_BATCH;
#endif
#ifdef SCHED_SPORADIC
      case RCL_THREAD_SCHEDULING_POLICY_SPORADIC:
        return SCHED_SPORADIC;
#endif
      /* Todo: Necessity and setting method need to be considered
      #ifdef SCHED_DEADLINE
          case RCL_THREAD_SCHEDULING_POLICY_DEADLINE:
            return SCHED_DEADLINE;
            break;
      #endif
      */
      default:
        throw std::runtime_error("Invalid scheduling policy");
    }
    return -1;
  }
};

}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__THREADS__POSIX__THREAD_ATTRIBUTE_HPP_
