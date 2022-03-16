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
#include <utility>

#include "rclcpp/rclcpp.hpp"

class PubNode : public rclcpp::Node
{
public:
  explicit PubNode(const std::string & node_name)
  : Node(node_name)
  {
    using namespace std::chrono_literals;
    publisher_ = this->create_publisher<builtin_interfaces::msg::Time>(
      "current_time",
      rclcpp::SystemDefaultsQoS());
    auto topictimer_callback =
      [&]() -> void {
        auto loan_time_ = publisher_->borrow_loaned_message();
        loan_time_.get() = this->get_clock()->now();
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "pub: Current timestamp is : " <<
            std::to_string(loan_time_.get().sec) <<
            " seconds, " <<
            std::to_string(loan_time_.get().nanosec) <<
            " nanoseconds.");
        publisher_->publish(std::move(loan_time_));
      };
    timer_ = this->create_wall_timer(500ms, topictimer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<PubNode>("topic_pub");
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::NodeOptions options;
  auto context_ = std::make_shared<rclcpp::Context>();
  options.context(context_);


  executor_.add_node(node_);
  executor_.spin();

  rclcpp::shutdown();
  return 0;
}
