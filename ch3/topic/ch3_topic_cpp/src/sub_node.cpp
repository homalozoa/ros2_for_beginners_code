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

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

class SubNode : public rclcpp::Node
{
public:
  explicit SubNode(const std::string & node_name)
  : Node(node_name)
  {
    subsciber_ = this->create_subscription<builtin_interfaces::msg::Time>(
      "current_time",
      // rclcpp::SystemDefaultsQoS(),
      rclcpp::QoS(0).keep_all().transient_local().reliable(),
      std::bind(&SubNode::count_sub_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr subsciber_;
  void count_sub_callback(const std::shared_ptr<builtin_interfaces::msg::Time> msg)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Sub: Current timestamp is : " <<
        std::to_string(msg->sec) <<
        " seconds, " <<
        std::to_string(msg->nanosec) <<
        " nanoseconds.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<SubNode>("topic_sub");
  rclcpp::executors::SingleThreadedExecutor executor_;

  executor_.add_node(node_);
  executor_.spin();

  rclcpp::shutdown();
  return 0;
}
