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
        std::cout << "missing log";  // << std::flush;
        RCLCPP_INFO_STREAM(
          this->get_logger(), "pid is " << pid << ", thread id is " <<
            std::this_thread::get_id());
      };
    printimer_ = this->create_wall_timer(500ms, printimer_cb);
  }

private:
  rclcpp::TimerBase::SharedPtr printimer_;
};
}  // namespace ros_beginner

int main(int argc, char ** argv)
{
  // setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto logger_test = std::make_shared<ros_beginner::LoggerTest>("cpp_log_test");
  auto executor = rclcpp::executors::StaticSingleThreadedExecutor();
  executor.add_node(logger_test);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
