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
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/illuminance.hpp"

class SelFilter : public rclcpp::Node
{
public:
  explicit SelFilter(const std::string & node_name)
  : Node(node_name)
  {
    using namespace std::chrono_literals;
    this->pressure_.header.frame_id = "pressure_link";
    this->illuminance_.header.frame_id = "illuminance_link";
    this->pressure_.fluid_pressure = 98000;
    pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(
      "sync_pressure",
      rclcpp::SystemDefaultsQoS());
    illuminance_pub_ = this->create_publisher<sensor_msgs::msg::Illuminance>(
      "sync_illuminance",
      rclcpp::SystemDefaultsQoS());
    auto pubtimer_callback =
      [&]() -> void {
        builtin_interfaces::msg::Time ts(this->get_clock()->now());
        // ts.nanosec = 0;
        this->pressure_.header.stamp = ts;
        this->pressure_.fluid_pressure += 0.01;
        ts.sec += 1;
        this->illuminance_.header.stamp = ts;
        this->illuminance_.illuminance += 0.01;
        this->pressure_pub_->publish(std::move(this->pressure_));
        this->illuminance_pub_->publish(std::move(this->illuminance_));
      };
    pub_timer_ = this->create_wall_timer(500ms, pubtimer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr pub_timer_;
  sensor_msgs::msg::FluidPressure pressure_;
  sensor_msgs::msg::Illuminance illuminance_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr illuminance_pub_;
};

void message_ts(const std::string & TAG, const builtin_interfaces::msg::Time & ts)
{
  RCLCPP_INFO(rclcpp::get_logger(TAG), "sec: %u, nanosec: %u.", ts.sec, ts.nanosec);
}

void illuminance_cb(const sensor_msgs::msg::Illuminance::SharedPtr & illuminance)
{
  message_ts("ILLUMIAN_CB", illuminance->header.stamp);
}

void pressure_cb(const sensor_msgs::msg::FluidPressure::SharedPtr & pressure)
{
  message_ts("PRESSURE_CB", pressure->header.stamp);
}

void ts_cb(
  const sensor_msgs::msg::Illuminance::SharedPtr & illuminance,
  const sensor_msgs::msg::FluidPressure::SharedPtr & pressure)
{
  message_ts("TS_ILLUMIAN", illuminance->header.stamp);
  message_ts("TS_PRESSURE", pressure->header.stamp);
}

#include "message_filters/subscriber.h"
#include "message_filters/cache.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelFilter>("selfilter");

  message_filters::Subscriber<sensor_msgs::msg::Illuminance> sub_i(node, "sync_illuminance");
  sub_i.registerCallback(illuminance_cb);
  // message_filters::Cache<sensor_msgs::msg::Illuminance> cache_i(sub_i, 10);
  // cache_i.registerCallback(illuminance_cb);


  message_filters::Subscriber<sensor_msgs::msg::FluidPressure> sub_p(node, "sync_pressure");
  sub_p.registerCallback(pressure_cb);
  // message_filters::Cache<sensor_msgs::msg::FluidPressure> cache_p(10);
  // cache_p.connectInput(sub_p);

  // message_filters::TimeSynchronizer<
  //   sensor_msgs::msg::Illuminance,
  //   sensor_msgs::msg::FluidPressure> time_sync(100);
  // time_sync.connectInput(sub_i, sub_p);
  // time_sync.registerCallback(ts_cb);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Illuminance,
      sensor_msgs::msg::FluidPressure> ApproximatedT;
  ApproximatedT policy(10);
  message_filters::Synchronizer<ApproximatedT> app_sync(policy);
  app_sync.connectInput(sub_i, sub_p);
  app_sync.registerCallback(ts_cb);


  rclcpp::executors::StaticSingleThreadedExecutor executor_;
  executor_.add_node(node);
  executor_.spin();
  rclcpp::shutdown();
  return 0;
}
