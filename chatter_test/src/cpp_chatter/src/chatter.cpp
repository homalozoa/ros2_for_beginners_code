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
#include <string>
#include <thread>

#include "cpp_chatter/chatter.hpp"

namespace ros_beginner
{
using namespace std::chrono_literals;
Chatter::Chatter(const std::string & chatter_name)
: rclcpp::Node(chatter_name)
{
  auto printimer_callback =
    [&]() -> void {
      pid_t pid = getpid();
      RCLCPP_INFO_STREAM(
        this->get_logger(), "pid is " << pid << ", thread id is " <<
          std::this_thread::get_id());
    };
  printimer_ = this->create_wall_timer(500ms, printimer_callback);
}

Chatter::~Chatter()
{}

}  // namespace ros_beginner
