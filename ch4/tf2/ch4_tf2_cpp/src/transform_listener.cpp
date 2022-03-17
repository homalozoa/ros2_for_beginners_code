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
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class TransformListener : public rclcpp::Node
{
public:
  explicit TransformListener(const std::string & node_name)
  : Node(node_name)
  {
    using namespace std::chrono_literals;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_timer_ = this->create_wall_timer(500ms, std::bind(&TransformListener::get_tf, this));
    this->declare_parameter("tolerance_time", 500);
    tolerance_time_ = this->get_parameter("tolerance_time").as_int();
    para_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TransformListener::param_cb, this, std::placeholders::_1));
  }

private:
  rcl_interfaces::msg::SetParametersResult param_cb(const std::vector<rclcpp::Parameter> & parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "tolerance_time") {
        tolerance_time_ = parameter.as_int();
      }
    }
    return result;
  }
  void get_tf()
  {
    geometry_msgs::msg::TransformStamped tf_local_;
    try {
      tf_local_ = tf_buffer_->lookupTransform(
        "robot", "world",
        this->get_clock()->now(),
        rclcpp::Duration(std::chrono::milliseconds(tolerance_time_)));
      RCLCPP_INFO_STREAM(
        this->get_logger(),
        "Got transform from " << tf_local_.child_frame_id <<
          " to " << tf_local_.header.frame_id <<
          " at " << std::to_string(tf_local_.header.stamp.sec) << " | " <<
          " x: " << std::to_string(tf_local_.transform.translation.x) <<
          " y: " << std::to_string(tf_local_.transform.translation.y) <<
          " z: " << std::to_string(tf_local_.transform.translation.z) <<
          " qx: " << std::to_string(tf_local_.transform.rotation.x) <<
          " qy: " << std::to_string(tf_local_.transform.rotation.y) <<
          " qz: " << std::to_string(tf_local_.transform.rotation.z) <<
          " qw: " << std::to_string(tf_local_.transform.rotation.w));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform robot to world: %s", ex.what());
      return;
    }
  }
  int32_t tolerance_time_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr para_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransformListener>("tf_listener"));
  rclcpp::shutdown();
  return 0;
}
