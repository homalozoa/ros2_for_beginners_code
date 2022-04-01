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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

int main()
{
  const auto LOGTAG = std::string("OperateBag");
  using TimeT = builtin_interfaces::msg::Time;
  TimeT time = rclcpp::Clock().now();
  auto rosbag_dir = rcpputils::fs::path("time_box");
  rclcpp::Serialization<TimeT> serialization;
  rcpputils::fs::remove_all(rosbag_dir);
  {
    rosbag2_cpp::Writer writer;
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    auto write_bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    serialization.serialize_message(&time, serialized_msg.get());  // set a time
    auto time2 = time;
    time2.nanosec += time.nanosec;
    serialization.serialize_message(&time2, serialized_msg.get());  // set another time
    writer.open(rosbag_dir.string());
    if (rcutils_system_time_now(&write_bag_msg->time_stamp) != RCL_RET_OK) {
      RCLCPP_ERROR(rclcpp::get_logger(LOGTAG), "Get time failed.");
      return 1;
    }
    // set metadata
    rosbag2_storage::TopicMetadata metadata;
    metadata.name = "/current_time";
    metadata.type = "builtin_interfaces/msg/Time";
    metadata.serialization_format = "cdr";
    writer.create_topic(metadata);

    // set bag message
    write_bag_msg->topic_name = metadata.name;
    write_bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      &serialized_msg->get_rcl_serialized_message(), [](rcutils_uint8_array_t *) {});

    // write bag message to bag directly
    writer.write(write_bag_msg);
    try {
      // false usage
      writer.write(write_bag_msg, "/next_time", "builtin_interfaces/msg/Time");
    } catch (const std::runtime_error & e) {
      std::cerr << e.what() << '\n';
    }

    // write ROS message to bag with topic name
    writer.write(time, "/next_time", rclcpp::Clock().now());

    // write serialized message with topic name
    #ifdef DEPRECATED_BAG_API
    writer.write(
      *serialized_msg, "/current_next_time", "builtin_interfaces/msg/Time",
      rclcpp::Clock().now());
    #else
    writer.write(
      serialized_msg, "/current_next_time", "builtin_interfaces/msg/Time",
      rclcpp::Clock().now());
    #endif
    RCLCPP_INFO(rclcpp::get_logger(LOGTAG), "Bag is wroten.");
  }
  {
    rosbag2_cpp::Reader reader;
    reader.open(rosbag_dir.string());
    std::vector<std::string> topic_names;
    while (reader.has_next()) {
      auto read_bag_msg = reader.read_next();
      topic_names.push_back(read_bag_msg->topic_name);

      TimeT ext_msg;
      rclcpp::SerializedMessage ext_serial_msg(*read_bag_msg->serialized_data);
      serialization.deserialize_message(
        &ext_serial_msg, &ext_msg);
      if (ext_msg == time) {
        RCLCPP_INFO(rclcpp::get_logger(LOGTAG), "Got time once");
      }
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(LOGTAG),
        "Topic name is " << topic_names[topic_names.size() - 1]);
    }
    RCLCPP_INFO(rclcpp::get_logger(LOGTAG), "Bag is read.");
  }
  return 0;
}
