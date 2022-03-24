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

#include "cpp_chatter/chatter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto current_time = std::chrono::time_point_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now());
  // auto chatter_node_1 = std::make_shared<ros_beginner::Chatter>("cpp_chatter1");
  auto count = current_time.time_since_epoch().count();
  std::cout << std::to_string(count) << std::endl;
  rclcpp::init(argc, argv);
  // auto chatter_node_1 = std::make_shared<ros_beginner::Chatter>("cpp_chatter1");
  // auto chatter_node_2 = std::make_shared<ros_beginner::Chatter>("cpp_chatter2");
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(chatter_node_1);
  // executor.add_node(chatter_node_2);
  // executor.spin();
  // for (int i = 0; i < 5; i++) {
  //   RCLCPP_INFO_STREAM_ONCE(
  //     chatter_node_1->get_logger(), "[info-once] outside log, flag: " << std::to_string(i));
  //   RCLCPP_INFO_STREAM(
  //     chatter_node_1->get_logger(),
  //     "[info-stream] outside log, flag: " << std::to_string(i));
  //   RCLCPP_INFO_STREAM_SKIPFIRST(
  //     chatter_node_1->get_logger(),
  //     "[info-stream-skipfirst] outside log, flag: " << std::to_string(i));
  // }
  rclcpp::shutdown();
  return 0;
}
