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

#ifndef RCLCPP__THREADS__POSIX__LINUX__CPU_SET_HPP_
#define RCLCPP__THREADS__POSIX__LINUX__CPU_SET_HPP_

#include <pthread.h>
#include <vector>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

namespace detail
{

struct CpuSet
{
  using NativeCpuSetType = cpu_set_t;
  CpuSet()
  {
    int processor_num = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    cpu_size_ = CPU_ALLOC_SIZE(processor_num);
    cpu_set_ = CPU_ALLOC(cpu_size_);
    CPU_ZERO_S(cpu_size_, cpu_set_);
  }
  explicit CpuSet(int num_cpu)
  {
    int processor_num = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    cpu_size_ = CPU_ALLOC_SIZE(processor_num);
    cpu_set_ = CPU_ALLOC(cpu_size_);
    CPU_ZERO_S(cpu_size_, cpu_set_);
    CPU_SET_S(num_cpu, cpu_size_, cpu_set_);
  }
  CpuSet(const CpuSet & cpuset)
  {
    int processor_num = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    cpu_size_ = cpuset.get_cpu_size();
    cpu_set_ = CPU_ALLOC(cpu_size_);
    CPU_ZERO_S(cpu_size_, cpu_set_);
    for (int i = 0; i < processor_num; i++) {
      if (CPU_ISSET_S(i, cpu_size_, cpuset.native_cpu_set())) {
        CPU_SET_S(i, cpu_size_, cpu_set_);
      }
    }
  }
  CpuSet & operator=(CpuSet const & cpuset)
  {
    int processor_num = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    cpu_size_ = cpuset.get_cpu_size();
    cpu_set_ = CPU_ALLOC(cpu_size_);
    CPU_ZERO_S(cpu_size_, cpu_set_);
    for (int i = 0; i < processor_num; i++) {
      if (CPU_ISSET_S(i, cpu_size_, cpuset.native_cpu_set())) {
        CPU_SET_S(i, cpu_size_, cpu_set_);
      }
    }
    return *this;
  }
  CpuSet(CpuSet &&) = delete;
  CpuSet & operator=(CpuSet &&) = delete;
  ~CpuSet()
  {
    CPU_FREE(cpu_set_);
    cpu_size_ = 0;
  }
  void set(int cpu)
  {
    int processor_num = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    if (0 > cpu || processor_num <= cpu) {
      auto ec = std::make_error_code(std::errc::invalid_argument);
      throw std::system_error{ec, "cpu number is invaild"};
    }
    CPU_SET_S(cpu, cpu_size_, cpu_set_);
  }
  void unset(int cpu)
  {
    int processor_num = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    if (0 > cpu || processor_num <= cpu) {
      auto ec = std::make_error_code(std::errc::invalid_argument);
      throw std::system_error{ec, "cpu number is invaild"};
    }
    CPU_CLR_S(cpu, cpu_size_, cpu_set_);
  }
  void clear()
  {
    CPU_ZERO_S(cpu_size_, cpu_set_);
  }
  bool is_set(int cpu)
  {
    return CPU_ISSET_S(cpu, cpu_size_, cpu_set_);
  }
  size_t get_cpu_size() const
  {
    return cpu_size_;
  }
  NativeCpuSetType * native_cpu_set() const {return cpu_set_;}

private:
  NativeCpuSetType * cpu_set_;
  size_t cpu_size_;
};
}  // namespace detail

}  // namespace rclcpp

#endif  // RCLCPP__THREADS__POSIX__LINUX__CPU_SET_HPP_
