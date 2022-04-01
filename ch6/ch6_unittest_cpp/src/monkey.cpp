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

#include "ch6_unittest_cpp/monkey.hpp"

namespace zoo
{
MonkeyNode::MonkeyNode(const std::string & node_name, const int32_t count_init)
: Node(node_name)
{
  count_bananas_ = count_init;
}

bool MonkeyNode::add_bananas(const int32_t & bananas)
{
  count_bananas_ += bananas;
  return true;
}

bool MonkeyNode::eat_bananas(const int32_t & bananas)
{
  bool rtn;
  if (count_bananas_ - bananas >= 0) {
    count_bananas_ -= bananas;
    rtn = true;
  } else {
    rtn = false;
  }
  return rtn;
}

int32_t MonkeyNode::check_bananas()
{
  return count_bananas_;
}
}  // namespace zoo
