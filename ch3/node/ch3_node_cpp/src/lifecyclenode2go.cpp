// Copyright (c) 2022 Homalozoa
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

#include <chrono>
#include <string>
#include <thread>

#include "ch3_node_cpp/lifecyclenode2go.hpp"

namespace ros_beginner
{
using namespace std::chrono_literals;
LifecycleNode2Go::LifecycleNode2Go(const std::string & node_name)
: rclcpp_lifecycle::LifecycleNode(node_name)
{}

LifecycleNode2Go::~LifecycleNode2Go()
{}

CallbackReturn_T LifecycleNode2Go::on_configure(const rclcpp_lifecycle::State &)
{
  std::cout << "Configuring node [" << this->get_name() << "]." << std::endl;
  auto printimer_callback =
    [&]() -> void {
      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        pid_t pid = getpid();
        std::cout << this->get_name() << ": pid is " << pid << ", thread id is " <<
          std::this_thread::get_id() << std::endl;
      }
    };
  printimer_ = this->create_wall_timer(500ms, printimer_callback);
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T LifecycleNode2Go::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::cout << "Cleaning up node [" << this->get_name() << "]." << std::endl;
  printimer_->cancel();
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T LifecycleNode2Go::on_shutdown(const rclcpp_lifecycle::State &)
{
  std::cout << "Shutting down node [" << this->get_name() << "]." << std::endl;
  if (!printimer_->is_canceled()) {
    printimer_->cancel();
  }
  printimer_->reset();
  return CallbackReturn_T::SUCCESS;
}

}  // namespace ros_beginner