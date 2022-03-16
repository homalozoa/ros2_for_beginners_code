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

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <turtlesim/msg/pose.hpp>

using std::placeholders::_1;

class DynamicTransform : public rclcpp::Node
{
public:
  DynamicTransform()
  : Node("turtle_tf2_frame_publisher")
  {
    // // Declare and acquire `turtlename` parameter
    // this->declare_parameter<std::string>("turtlename", "turtle");
    // this->get_parameter("turtlename", turtlename_);

    // // Initialize the transform broadcaster
    // tf_broadcaster_ =
    //   std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // // callback function on each message
    // std::ostringstream stream;
    // stream << "/" << turtlename_.c_str() << "/pose";
    // std::string topic_name = stream.str();

    // subscription_ = this->create_subscription<turtlesim::msg::Pose>(
    //   topic_name, 10,
    //   std::bind(&DynamicTransform::handle_turtle_pose, this, _1));
  }

private:
  // void handle_turtle_pose(const turtlesim::msg::Pose & msg)
  // {
  //   rclcpp::Time now = this->get_clock()->now();
  //   geometry_msgs::msg::TransformStamped t;

  //   // Read message content and assign it to
  //   // corresponding tf variables
  //   t.header.stamp = now;
  //   t.header.frame_id = "world";
  //   t.child_frame_id = turtlename_.c_str();

  //   // Turtle only exists in 2D, thus we get x and y translation
  //   // coordinates from the message and set the z coordinate to 0
  //   t.transform.translation.x = msg.x;
  //   t.transform.translation.y = msg.y;
  //   t.transform.translation.z = 0.0;

  //   // For the same reason, turtle can only rotate around one axis
  //   // and this why we set rotation in x and y to 0 and obtain
  //   // rotation in z axis from the message
  //   tf2::Quaternion q;
  //   q.setRPY(0, 0, msg.theta);
  //   t.transform.rotation.x = q.x();
  //   t.transform.rotation.y = q.y();
  //   t.transform.rotation.z = q.z();
  //   t.transform.rotation.w = q.w();

  //   // Send the transformation
  //   tf_broadcaster_->sendTransform(t);
  // }
  // rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // std::string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTransform>());
  rclcpp::shutdown();
  return 0;
}
