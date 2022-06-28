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

#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

class NodeWithOption : public rclcpp::Node
{
public:
  NodeWithOption(const std::string & node_name, const rclcpp::NodeOptions & options)
  : rclcpp::Node(node_name, options)
  {
    if (options.use_global_arguments()) {
      RCLCPP_INFO_STREAM(this->get_logger(), std::string("Using global arguments"));
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), std::string("Not using global arguments"));
    }
    RCLCPP_INFO_STREAM(this->get_logger(), this->get_namespace());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto exec = rclcpp::executors::SingleThreadedExecutor();
  auto options = rclcpp::NodeOptions();
  options.use_global_arguments(atoi(argv[2]));

  auto node = std::make_shared<NodeWithOption>(argv[1], options);
  exec.add_node(node->get_node_base_interface());
  exec.spin_once();

  rclcpp::shutdown();
}
