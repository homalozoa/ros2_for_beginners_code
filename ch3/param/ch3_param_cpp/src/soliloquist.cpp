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

#include "rclcpp/rclcpp.hpp"

namespace ros_beginner
{

class Soliloquist : public rclcpp::Node
{
public:
  explicit Soliloquist()
  : Node("cpp_soliloquist")
  {
    this->declare_parameter<int>("time_cycle_ms", 500);
    this->declare_parameter<std::string>("output_str", "hello_world");
    time_cycle_ms_ = this->get_parameter("time_cycle_ms").as_int();
    output_str_ = std::make_shared<std::string>(this->get_parameter("output_str").as_string());
    auto printimer_cb =
      [&]() -> void {
        RCLCPP_INFO_STREAM(this->get_logger(), *output_str_);
      };
    auto param_cb =
      [&](const std::vector<rclcpp::Parameter> & parameters) -> rcl_interfaces::msg::
      SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "nothing changed";

        for (const auto & para : parameters) {
          RCLCPP_INFO_STREAM(this->get_logger(), para.get_name());
          if (para.get_name().compare("output_str") == 0) {
            if (!para.as_string().empty() && para.as_string().compare(*output_str_)) {
              output_str_ = std::make_shared<std::string>(para.as_string());
              RCLCPP_WARN_STREAM(this->get_logger(), "Output string changed.");
              result.successful = true;
              result.reason = "success";
            }
          }
        }

        return result;
      };
    printimer_ = this->create_wall_timer(std::chrono::milliseconds(time_cycle_ms_), printimer_cb);
    paramhandle_ = this->add_on_set_parameters_callback(param_cb);
  }

private:
  int time_cycle_ms_;
  std::shared_ptr<std::string> output_str_;
  rclcpp::TimerBase::SharedPtr printimer_;
  OnSetParametersCallbackHandle::SharedPtr paramhandle_;
};
}  // namespace ros_beginner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_ = std::make_shared<ros_beginner::Soliloquist>();
  auto exec_ = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
  exec_->add_node(node_->get_node_base_interface());
  exec_->spin();
  rclcpp::shutdown();
  return 0;
}
