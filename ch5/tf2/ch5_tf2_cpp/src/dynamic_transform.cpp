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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class DynamicTransform : public rclcpp::Node
{
public:
  explicit DynamicTransform(const std::string & node_name)
  : Node(node_name)
  {
    using namespace std::chrono_literals;
    delta_ = 0.0;
    tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_timer_ =
      this->create_wall_timer(30ms, std::bind(&DynamicTransform::update_transform, this));
  }

private:
  void update_transform()
  {
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped trans;
    tf2::Quaternion quat;

    trans.header.stamp = now;
    trans.header.frame_id = "map";
    trans.child_frame_id = "robot";
    trans.transform.translation.x = delta_;
    trans.transform.translation.y = 0;
    trans.transform.translation.z = 0;
    quat.setRPY(0, 0, 0);
    trans.transform.rotation.x = quat.x();
    trans.transform.rotation.y = quat.y();
    trans.transform.rotation.z = quat.z();
    trans.transform.rotation.w = quat.w();

    tf_publisher_->sendTransform(trans);
    delta_ += 0.01;
  }
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  double delta_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTransform>("dynamic_tf_node"));
  rclcpp::shutdown();
  return 0;
}
