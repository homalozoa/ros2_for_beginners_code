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

#include "ch3_plugin_base/pluginbase.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ch3
{
namespace plugin
{
class PluginAlphaA : public PluginBase
{
public:
  PluginAlphaA() {}
  void say_hello(const int32_t & times) override
  {
    auto times_(times);
    while (--times_ >= 0) {
      RCLCPP_INFO(rclcpp::get_logger("PluginAlphaA"), "Hello A alpha");
    }
  }
  bool say_something(const std::string & something) override
  {
    if (something.size() == 0) {
      return false;
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("PluginAlphaA"), something);
      return true;
    }
  }
};

class PluginAlphaB : public PluginBase
{
public:
  PluginAlphaB() {}
  void say_hello(const int32_t & times) override
  {
    auto times_(times);
    while (--times_ >= 0) {
      RCLCPP_INFO(rclcpp::get_logger("PluginAlphaB"), "Hello B alpha");
    }
  }
  bool say_something(const std::string & something) override
  {
    if (something.size() == 0) {
      return false;
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("PluginAlphaB"), something);
      return true;
    }
  }
};
}  // namespace plugin
}  // namespace ch3

PLUGINLIB_EXPORT_CLASS(ch3::plugin::PluginAlphaA, ch3::plugin::PluginBase)
PLUGINLIB_EXPORT_CLASS(ch3::plugin::PluginAlphaB, ch3::plugin::PluginBase)
