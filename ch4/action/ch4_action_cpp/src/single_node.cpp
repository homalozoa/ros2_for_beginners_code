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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ch4_action_interfaces/action/count.hpp"
#include "ch4_action_cpp/action_server.hpp"

class SingleNode : public rclcpp::Node
{
using CountT = ch4_action_interfaces::action::Count;
public:
  explicit SingleNode(const std::string & node_name)
  : Node(node_name)
  {
    using namespace std::chrono_literals;
    global_count_ = 0;
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_ = rclcpp_action::create_client<CountT>(this, "sec_count");
    auto clientimer_callback =
      [&]() -> void {
        auto goal = CountT::Goal();
        goal.goal_count = 3;
        client_->async_send_goal(std::move(goal));
      };
    clientimer_ = this->create_wall_timer(9s, clientimer_callback);
    this->server_ = std::make_unique<utils::ActionServer<CountT, Node>>(
      this->shared_from_this(),
      "sec_count",
      std::bind(&SingleNode::execution_cb, this));
  }

private:
  rclcpp::TimerBase::SharedPtr clientimer_;
  std::unique_ptr<utils::ActionServer<CountT, Node>> server_;
  rclcpp_action::Client<CountT>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  uint32_t global_count_;
  void execution_cb()
  {
    auto result = std::make_shared<CountT::Result>();
    auto feedback = std::make_shared<CountT::Feedback>();
    const auto goal = server_->get_current_goal();
    rclcpp::Rate loop_rate(1);
    uint32_t local_count = 0;
    RCLCPP_INFO_STREAM(this->get_logger(),
      "Got goal: " << std::to_string(goal->goal_count));
    while (local_count < goal->goal_count) {
      if (server_->is_cancel_requested()) {
        break;
      }
      feedback->local_count = local_count;
      server_->publish_feedback(feedback);
      local_count += 1;
      loop_rate.sleep();
    }
    global_count_ += local_count;
    result->global_count = global_count_;
    server_->succeeded_current(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto srv_node_ = std::make_shared<SingleNode>("self_action");
  rclcpp::executors::MultiThreadedExecutor executor_;

  executor_.add_node(srv_node_);
  executor_.spin();

  rclcpp::shutdown();
  return 0;
}
