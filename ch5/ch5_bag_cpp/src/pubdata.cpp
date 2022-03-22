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

class PubNode : public rclcpp::Node
{
public:
  explicit PubNode(const std::string & node_name)
  : Node(node_name)
  {
    using namespace std::chrono_literals;
    time_publisher_ = this->create_publisher<builtin_interfaces::msg::Time>(
      "current_time",
      rclcpp::SystemDefaultsQoS());
    duration_publisher_ = this->create_publisher<builtin_interfaces::msg::Duration>(
      "current_duration",
      rclcpp::SystemDefaultsQoS());
    ts_init_ = this->get_clock()->now();
    auto topictimer_callback =
      [&]() -> void {
        auto timestamp = std::make_unique<builtin_interfaces::msg::Time>(this->get_clock()->now());
        auto duration = std::make_unique<builtin_interfaces::msg::Duration>();
        duration->sec = timestamp->sec - ts_init_.seconds();
        duration->nanosec = timestamp->nanosec - ts_init_.nanoseconds();
        time_publisher_->publish(std::move(timestamp));
        duration_publisher_->publish(std::move(duration));
      };
    timer_ = this->create_wall_timer(500ms, topictimer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time ts_init_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_publisher_;
  rclcpp::Publisher<builtin_interfaces::msg::Duration>::SharedPtr duration_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PubNode>("topic_pub");
  rclcpp::executors::StaticSingleThreadedExecutor executor_;
  auto current_time = node->get_clock()->now();
  executor_.add_node(node);
  RCLCPP_INFO(node->get_logger(), "Begin.");
  while (node->get_clock()->now() - current_time <= std::chrono::milliseconds(10'000)) {
    executor_.spin_some();
  }
  RCLCPP_INFO(node->get_logger(), "End.");
  rclcpp::shutdown();
  return 0;
}
