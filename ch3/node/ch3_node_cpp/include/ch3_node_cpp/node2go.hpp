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

#ifndef CH3_NODE_CPP__NODE2GO_HPP_
#define CH3_NODE_CPP__NODE2GO_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace ros_beginner
{
class Node2Go : public rclcpp::Node
{
public:
  explicit Node2Go(const std::string & node_name);
  ~Node2Go();

private:
  rclcpp::TimerBase::SharedPtr printimer_;
};
}  // namespace ros_beginner
#endif  // CH3_NODE_CPP__NODE2GO_HPP_
