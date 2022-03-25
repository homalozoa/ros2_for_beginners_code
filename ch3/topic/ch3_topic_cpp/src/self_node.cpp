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

class SelfNode : public rclcpp::Node
{
public:
  explicit SelfNode(const std::string & node_name)
  : Node(node_name)
  {
    using namespace std::chrono_literals;
    subsciber_ = this->create_subscription<builtin_interfaces::msg::Time>(
      "current_time",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&SelfNode::count_sub_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<builtin_interfaces::msg::Time>(
      "current_time",
      rclcpp::SystemDefaultsQoS());
    auto topictimer_callback =
      [&]() -> void {
        timestamp_ = this->get_clock()->now();
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "pub: Current timestamp is : " <<
            std::to_string(timestamp_.sec) <<
            " second, " <<
            std::to_string(timestamp_.nanosec) <<
            " nanosecond.");
        publisher_->publish(timestamp_);
      };
    timer_ = this->create_wall_timer(1s, topictimer_callback);
  }

private:
  builtin_interfaces::msg::Time timestamp_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
  rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr subsciber_;
  void count_sub_callback(const std::shared_ptr<builtin_interfaces::msg::Time> msg)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Sub: Current timestamp is : " <<
        std::to_string(msg->sec) <<
        " second, " <<
        std::to_string(msg->nanosec) <<
        " nanosecond.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<SelfNode>("topic_self");
  rclcpp::executors::SingleThreadedExecutor executor_;

  executor_.add_node(node_);
  executor_.spin();

  rclcpp::shutdown();
  return 0;
}
