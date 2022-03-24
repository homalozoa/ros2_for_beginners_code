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

#include "ch5_unittest_cpp/monkey.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

class TestNode : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST(MonkeyRawTest, initBananas)
{
  rclcpp::init(0, nullptr);
  auto monkey_raw = std::make_shared<zoo::MonkeyNode>("first_monkey", 100);
  ASSERT_EQ(monkey_raw->check_bananas(), 100);
  ASSERT_STREQ(monkey_raw->get_name(), "first_monkey");
  rclcpp::shutdown();
}

TEST(MonkeyRawTest, faileInitMonkey)
{
  ASSERT_THROW(std::make_shared<zoo::MonkeyNode>("another_monkey"), rclcpp::exceptions::RCLError);
}

TEST_F(TestNode, setBananas)
{
  auto monkey_raw = std::make_unique<zoo::MonkeyNode>("second_monkey");
  EXPECT_TRUE(monkey_raw->add_bananas(99));
  ASSERT_EQ(monkey_raw->check_bananas(), 99);
}

TEST_F(TestNode, eatAfterInitAndSet)
{
  auto monkey_raw = std::make_unique<zoo::MonkeyNode>("third_monkey", 10);
  EXPECT_TRUE(monkey_raw->add_bananas(20));
  EXPECT_FALSE(monkey_raw->eat_bananas(100));
  EXPECT_EQ(monkey_raw->check_bananas(), 30);
  EXPECT_TRUE(monkey_raw->eat_bananas(15));
  ASSERT_EQ(monkey_raw->check_bananas(), 15);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
