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
#include <utility>

#include "rclcpp/rclcpp.hpp"

class SingleNode : public rclcpp::Node
{
public:
  explicit SingleNode(const std::string & node_name)
  : Node(node_name)
  {
    using namespace std::chrono_literals;
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_ = this->create_client<rcl_interfaces::srv::GetParameters>("get_para");
    auto clientimer_callback =
      [&]() -> void {
        auto req = std::make_unique<rcl_interfaces::srv::GetParameters::Request>();
        req->names.push_back("abc");
        RCLCPP_INFO_STREAM(this->get_logger(), "Ready to send req");
        client_->async_send_request(std::move(req));
        RCLCPP_INFO_STREAM(this->get_logger(), "Sent req");
      };
    clientimer_ = this->create_wall_timer(500ms, clientimer_callback);
    auto logtimer_callback =
      [&]() -> void {
        RCLCPP_INFO_STREAM(this->get_logger(), "Hello earth");
      };
    logtimer_ = this->create_wall_timer(500ms, logtimer_callback);
    server_ = this->create_service<rcl_interfaces::srv::GetParameters>(
      "get_para",
      std::bind(&SingleNode::service_callback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      cb_group_);
  }

private:
  rclcpp::TimerBase::SharedPtr clientimer_;
  rclcpp::TimerBase::SharedPtr logtimer_;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr server_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  void service_callback(
    const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
    std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response)
  {
    rcl_interfaces::msg::Parameter para_;
    para_.value.bool_value = !request->names.empty();
    response->values.push_back(para_.value);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Response: " << std::to_string(response->values[0].bool_value));
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto srv_node_ = std::make_shared<SingleNode>("srv_self");
  rclcpp::executors::MultiThreadedExecutor executor_;

  executor_.add_node(srv_node_);
  executor_.spin();

  rclcpp::shutdown();
  return 0;
}
