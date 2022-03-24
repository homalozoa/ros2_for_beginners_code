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

#ifndef CH5_UNITTEST_CPP__MONKEY_HPP_
#define CH5_UNITTEST_CPP__MONKEY_HPP_
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace zoo
{
class MonkeyNode : public rclcpp::Node
{
public:
  explicit MonkeyNode(const std::string & node_name, const int32_t count_init = 0);
  bool add_bananas(const int32_t & bananas);
  bool eat_bananas(const int32_t & bananas);
  int32_t check_bananas();

private:
  int32_t count_bananas_;
};
}  // namespace zoo
#endif  // CH5_UNITTEST_CPP__MONKEY_HPP_
