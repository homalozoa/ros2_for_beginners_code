// Copyright 2022 Homalozoa
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

#ifndef CPP_CHATTER__LIFECYCLE_CHATTER_HPP_
#define CPP_CHATTER__LIFECYCLE_CHATTER_HPP_

#include <unistd.h>

#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sys/types.h"

namespace ros_beginner
{
using CallbackReturn_T = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
class LifecycleChatter : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleChatter(const std::string & chatter_name);
  ~LifecycleChatter();

  // Lifecycle functions
  CallbackReturn_T on_configure(const rclcpp_lifecycle::State & state) override;
  // CallbackReturn_T on_activate(const rclcpp_lifecycle::State &);
  // CallbackReturn_T on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn_T on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn_T on_shutdown(const rclcpp_lifecycle::State &);

private:
  rclcpp::TimerBase::SharedPtr printimer_;
  void test_func(){}
};
}  // namespace ros_beginner
#endif  // CPP_CHATTER__LIFECYCLE_CHATTER_HPP_
