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

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  using namespace std::chrono_literals;
  auto node_a = rclcpp::Node("TestNode");
  auto executor = rclcpp::executors::StaticSingleThreadedExecutor();
  int * int_ptr;
  auto timer_cb = [&]() -> void {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("DebugTest"), std::to_string(*int_ptr));
    };
  auto timer_ = node_a.create_wall_timer(1s, timer_cb);
  executor.add_node(node_a.get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
