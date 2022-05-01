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
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "diagnostic_updater/update_functions.hpp"
#include "sensor_msgs/msg/range.hpp"

void tick_freq(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Tick tick");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_n = rclcpp::Node::make_shared("n_node");
  auto exec = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();

  diagnostic_updater::Updater updater_n(node_n);
  updater_n.setHardwareID("normal");
  double min_freq(29.0);
  double max_freq(31.0);
  diagnostic_updater::HeaderlessTopicDiagnostic hdls_freq_checker(
    "rate_freq_1", updater_n,
    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
  diagnostic_updater::FunctionDiagnosticTask freq_task(
    "Tick frequncy checker task", std::bind(&tick_freq, std::placeholders::_1));
  hdls_freq_checker.addTask(&freq_task);

  diagnostic_updater::FrequencyStatusParam freq_param(&min_freq, &max_freq);
  diagnostic_updater::TimeStampStatusParam ts_param(-1.0, 3.0);
  diagnostic_updater::TopicDiagnostic freq_checker("rate_freq_2", updater_n, freq_param, ts_param);

  auto range_pub =
    node_n->create_publisher<sensor_msgs::msg::Range>("range", rclcpp::SystemDefaultsQoS());
  diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::Range> diag_pub(range_pub, updater_n,
    freq_param, ts_param);
  sensor_msgs::msg::Range range;

  exec->add_node(node_n);
  rclcpp::Rate r(30);
  while (rclcpp::ok()) {
    exec->spin_some();
    hdls_freq_checker.tick();
    // range.header.stamp = node_n->get_clock()->now();
    // freq_checker.tick(range.header.stamp);
    // diag_pub.publish(range);
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
