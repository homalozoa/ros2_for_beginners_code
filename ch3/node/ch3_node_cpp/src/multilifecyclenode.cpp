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
#include <vector>
#include <string>

#include "ch3_node_cpp/lifecyclenode2go.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  uint32_t node_count(0);
  bool is_multi(false);
  std::vector<std::shared_ptr<ros_beginner::LifecycleNode2Go>> node_vector;
  rclcpp::executors::SingleThreadedExecutor executor_s;
  rclcpp::executors::MultiThreadedExecutor executor_m;

  if (argc >= 3) {
    int input_count = atoi(argv[1]);
    node_count = input_count > 0 ? input_count : 0;
    node_vector.reserve(node_count);
    std::string multi_flag = static_cast<std::string>(argv[2]);
    if (multi_flag == std::string("m")) {
      is_multi = true;
    } else if (multi_flag == std::string("s")) {
      is_multi = false;
    } else {
      std::cout << "Example: ros2 run ch3_node_cpp multilifecyclenode <node_count> s/m" << std::endl;
      return 0;
    }
  } else {
    std::cout << "Example: ros2 run ch3_node_cpp multilifecyclenode <node_count> s/m" << std::endl;
    return 0;
  }

  for (int i = node_count; i--; ) {
    node_vector.push_back(
      std::make_shared<ros_beginner::LifecycleNode2Go>(
        "cpp_node_a_" +
        std::to_string(i)));
    if (is_multi) {executor_m.add_node(node_vector.back()->get_node_base_interface());} else {
      executor_s.add_node(node_vector.back()->get_node_base_interface());
    }
  }
  if (is_multi) {executor_m.spin();} else {executor_s.spin();}

  rclcpp::shutdown();
  return 0;
}
