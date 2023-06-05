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

#include <iostream>
#include "rclcpp/threads/posix/thread.hpp"

static void * thread_main(void * p);

namespace rclcpp
{

Thread::Thread(Attribute * attr, ThreadFuncUniquePtr func)
: handle_(NativeHandleType{})
{
  Attribute::NativeAttributeType native_attr;
  int r = pthread_attr_init(&native_attr);
  if (r != 0) {throw std::system_error(r, std::system_category(), "Error in pthread_attr_init ");}

  if (attr != nullptr) {
    r = pthread_attr_setinheritsched(&native_attr, PTHREAD_EXPLICIT_SCHED);
    if (r != 0) {
      throw std::system_error(r, std::system_category(), "Error in pthread_attr_setinheritsched ");
    }

    rclcpp::detail::CpuSet affinity = attr->get_affinity();
    size_t cpu_size_ = CPU_ALLOC_SIZE(static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN)));
    r = pthread_attr_setaffinity_np(&native_attr, cpu_size_, affinity.native_cpu_set());
    if (r != 0) {
      throw std::system_error(r, std::system_category(), "Error in pthread_attr_setaffinity_np ");
    }

    std::size_t stack_size = attr->get_stack_size();
    r = pthread_attr_setstacksize(&native_attr, stack_size);
    if (r != 0) {
      throw std::system_error(r, std::system_category(), "Error in pthread_attr_setstacksize ");
    }

    int flag = attr->get_run_as_detached() ? PTHREAD_CREATE_DETACHED : PTHREAD_CREATE_JOINABLE;
    r = pthread_attr_setdetachstate(&native_attr, flag);
    if (r != 0) {
      throw std::system_error(r, std::system_category(), "Error in pthread_attr_setdetachstate ");
    }

    int sched_policy = attr->get_sched_policy();
    if (sched_policy == SCHED_FIFO || sched_policy == SCHED_RR) {
      r = pthread_attr_setschedpolicy(&native_attr, sched_policy);
      if (r != 0) {
        throw std::system_error(r, std::system_category(), "Error in pthread_attr_setschedpolicy ");
      }

      sched_param param;
      param.sched_priority = attr->get_priority();
      r = pthread_attr_setschedparam(&native_attr, &param);
      if (r != 0) {
        throw std::system_error(r, std::system_category(), "Error in pthread_attr_setschedparam ");
      }
    }
  }

  NativeHandleType h;

  r = pthread_create(&h, &native_attr, thread_main, func.get());
  if (r != 0) {throw std::system_error(r, std::system_category(), "Error in pthread_create ");}

  if (attr == nullptr || !attr->get_run_as_detached()) {
    this->handle_ = h;
  }

  pthread_attr_destroy(&native_attr);

  func.release();
}

}  // namespace  rclcpp

static void * thread_main(void * p)
{
  using rclcpp::detail::ThreadFuncUniquePtr;
  using rclcpp::detail::ThreadFunc;

  ThreadFuncUniquePtr func{reinterpret_cast<ThreadFunc *>(p)};
  try {
    func->run();
  } catch (...) {
    std::cerr << "failed to run thread" << std::endl;
    std::terminate();
  }

  return nullptr;
}
