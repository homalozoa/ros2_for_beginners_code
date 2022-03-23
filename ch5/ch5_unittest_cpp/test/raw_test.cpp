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

#include "ch5_unittest_cpp/monkey_raw.hpp"
#include "gtest/gtest.h"

TEST(MonkeyRawTest, initBananas)
{
  auto monkey_raw = zoo::MonkeyNode(100);
  ASSERT_EQ(monkey_raw.get_bananas(), 100);
}

TEST(MonkeyRawTest, setBananas)
{
  auto monkey_raw = zoo::MonkeyNode();
  EXPECT_TRUE(monkey_raw.add_bananas(99));
  ASSERT_EQ(monkey_raw.get_bananas(), 99);
}

TEST(MonkeyRawTest, eatAfterInitAndSet)
{
  auto monkey_raw = zoo::MonkeyNode(10);
  EXPECT_TRUE(monkey_raw.add_bananas(20));
  EXPECT_FALSE(monkey_raw.eat_bananas(100));
  EXPECT_EQ(monkey_raw.get_bananas(), 30);
  EXPECT_TRUE(monkey_raw.eat_bananas(15));
  ASSERT_EQ(monkey_raw.get_bananas(), 15);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
