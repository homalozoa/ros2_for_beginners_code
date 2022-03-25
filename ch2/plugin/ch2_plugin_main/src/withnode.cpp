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

#include <iostream>
#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "ch2_plugin_base/pluginbase.hpp"
#include "rclcpp/rclcpp.hpp"

class TestNode : public rclcpp::Node
{
public:
  explicit TestNode(const std::string & node_name)
  : Node(node_name)
  {
    using namespace std::chrono_literals;
    this->declare_parameter("plugin_name", "AlphaB");
    loader = std::make_shared<pluginlib::ClassLoader<ch2::plugin::PluginBase>>(
      "ch2_plugin_base",
      "ch2::plugin::PluginBase");
    auto printimer_callback =
      [&]() -> void {
        auto plugin_name = this->get_parameter("plugin_name").as_string();
        try {
          auto plugin_instance = loader->createUniqueInstance(plugin_name);
          plugin_instance->say_hello(1);
        } catch (pluginlib::PluginlibException & ex) {
          printf("Failed to load PluginBeta. Error: %s\n", ex.what());
        }
      };
    timer_ = this->create_wall_timer(500ms, printimer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<pluginlib::ClassLoader<ch2::plugin::PluginBase>> loader;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<TestNode>("plugin_test_node");
  rclcpp::executors::SingleThreadedExecutor executor_;

  executor_.add_node(node_);
  executor_.spin();

  rclcpp::shutdown();
  return 0;
}
