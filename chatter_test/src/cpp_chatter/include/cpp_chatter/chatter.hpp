// Copyright 2022 Homalozoa
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

#ifndef CPP_CHATTER__CHATTER_HPP_
#define CPP_CHATTER__CHATTER_HPP_

#include <unistd.h>

#include <string>
#include <string_view>  // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "sys/types.h"

namespace ros_beginner
{
class Chatter : public rclcpp::Node
{
public:
  explicit Chatter(const std::string & chatter_name);
  ~Chatter();

private:
  void message_debug(std::string_view tag, std::string_view debug)
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "[" << tag << "] " << debug);
  }

  void message_info(std::string_view tag, std::string_view info)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << tag << "] " << info);
  }
  rclcpp::TimerBase::SharedPtr printimer_;
};
}  // namespace ros_beginner
#endif  // CPP_CHATTER__CHATTER_HPP_
