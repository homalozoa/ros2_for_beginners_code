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

#include <unistd.h>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sys/types.h"

namespace ros_beginner
{
using namespace std::chrono_literals;

class LoggerTest : public rclcpp::Node
{
public:
  explicit LoggerTest(const std::string & node_name)
  : Node(node_name)
  {
    auto printimer_cb =
      [&]() -> void {
        pid_t pid = getpid();
        std::cout << this->get_name() << ": pid is " << pid << ", thread id is " <<
          std::this_thread::get_id() << std::endl;
      };
    printimer_ = this->create_wall_timer(500ms, printimer_cb);
    auto condition_func =
      [&](const bool cond) -> bool {
        return cond;
      };
    std::function<bool()> condition_func_true = std::bind(condition_func, true);
    std::function<bool()> condition_func_false = std::bind(condition_func, false);
    RCLCPP_INFO(this->get_logger(), "[info] inside log [%s]", this->get_name());
    RCLCPP_INFO_STREAM(this->get_logger(), "[info-stream] inside log [" << this->get_name() << "]");
    RCLCPP_INFO_FUNCTION(
      this->get_logger(),
      &condition_func_true,
      "[info-func] func log true output");
    RCLCPP_INFO_FUNCTION(
      this->get_logger(),
      &condition_func_false,
      "[info-func] func log false output");
  }

private:
  rclcpp::TimerBase::SharedPtr printimer_;
};
}  // namespace ros_beginner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto logger_test = std::make_shared<ros_beginner::LoggerTest>("cpp_log_test");
  for (int i = 0; i < 5; i++) {
    RCLCPP_INFO_STREAM_ONCE(
      logger_test->get_logger(), "[info-once] outside log, flag: " << std::to_string(i));
    RCLCPP_INFO_STREAM(
      logger_test->get_logger(),
      "[info-stream] outside log, flag: " << std::to_string(i));
    RCLCPP_INFO_STREAM_SKIPFIRST(
      logger_test->get_logger(),
      "[info-stream-skipfirst] outside log, flag: " << std::to_string(i));
  }
  rclcpp::shutdown();
  return 0;
}
