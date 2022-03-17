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

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticTransform : public rclcpp::Node
{
public:
  explicit StaticTransform(const std::string & node_name)
  // explicit StaticTransform(const std::string & node_name, char * argv[])
  : Node(node_name)
  {
    tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->set_transform(
      "world", "map",  // frame ids
      1, 1, 1,  // translation
      0, 0, 0);  // rotation
    // this->set_transform(
    //   argv[1], argv[2],  // frame ids
    //   atof(argv[3]), atof(argv[4]), atof(argv[5]),  // translation
    //   atof(argv[6]), atof(argv[7]), atof(argv[8]));  // rotation
  }

private:
  void set_transform(
    const std::string & source_frame,
    const std::string & target_frame,
    const double & trans_x,
    const double & trans_y,
    const double & trans_z,
    const double & rot_roll,
    const double & rot_pitch,
    const double & rot_yaw)
  {
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped trans;
    tf2::Quaternion quat;

    trans.header.stamp = now;
    trans.header.frame_id = source_frame;
    trans.child_frame_id = target_frame;
    trans.transform.translation.x = trans_x;
    trans.transform.translation.y = trans_y;
    trans.transform.translation.z = trans_z;
    quat.setRPY(rot_roll, rot_pitch, rot_yaw);
    trans.transform.rotation.x = quat.x();
    trans.transform.rotation.y = quat.y();
    trans.transform.rotation.z = quat.z();
    trans.transform.rotation.w = quat.w();

    tf_publisher_->sendTransform(trans);
  }
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};

int main(int argc, char * argv[])
{
  // if (argc != 9) {
  //   RCLCPP_ERROR(
  //     rclcpp::get_logger("Notice"),
  //     "Usage: ros2 run ch4_tf2_cpp static_transform header_frame child_frame x y z r p y");
  //   return 1;
  // }
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<StaticTransform>("static_tf_node", argv));
  rclcpp::spin(std::make_shared<StaticTransform>("static_tf_node"));
  rclcpp::shutdown();
  return 0;
}
