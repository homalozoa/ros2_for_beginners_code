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
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

std::shared_ptr<double> temp_sensor_a;
std::shared_ptr<double> temp_sensor_b;
std::shared_ptr<double> temp_sensor_c;

// void temp_sensor(diagnostic_updater::DiagnosticStatusWrapper & stat)
// {
//   auto sensor_value = *temp_sensor_a;
//   stat.add<double>("temperature", sensor_value);
//   if (sensor_value >= 100) {
//     stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "so hot");
//   } else if (sensor_value > 50 && sensor_value < 100) {
//     stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "a little hot, but tolerable");
//   } else {
//     stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "cool");
//   }
// }

void temp_sensor(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  auto sensor_value = *temp_sensor_a;
  diagnostic_msgs::msg::DiagnosticStatus status;

  if (sensor_value >= 100) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "so hot";
  } else if (sensor_value > 50 && sensor_value < 100) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "a little hot, but tolerable";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "cool";
  }
  stat.add<double>("temperature", sensor_value);
  stat.summary(status);
}

class TempSensor : public diagnostic_updater::DiagnosticTask
{
public:
  TempSensor(const std::string & task_name, const std::weak_ptr<double> sensor_ptr)
  : DiagnosticTask(task_name),
    sensor_ptr_(sensor_ptr.lock()) {}
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    std::string name("temp sensor B");
    auto sensor_value = *sensor_ptr_;
    if (sensor_value >= 100) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "so hot";
    } else if (sensor_value > 50 && sensor_value < 100) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "a little hot, but tolerable";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "cool";
    }
    stat.add<double>("temperature", sensor_value);
    stat.summary(status);
  }

private:
  std::shared_ptr<double> sensor_ptr_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_n = rclcpp::Node::make_shared("n_node");
  auto exec = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0, 150);

  diagnostic_updater::Updater updater_n(node_n);
  temp_sensor_a = std::make_shared<double>(0.0);
  temp_sensor_b = std::make_shared<double>(0.0);
  temp_sensor_c = std::make_shared<double>(0.0);

  updater_n.setHardwareID("normal");
  updater_n.add("/standalone/temp_1", temp_sensor);

  TempSensor sensor_2("/class/temp_2", temp_sensor_b);
  TempSensor sensor_11("/class/temp_1", temp_sensor_a);
  updater_n.add(sensor_2);
  updater_n.add(sensor_11);

  exec->add_node(node_n);
  rclcpp::Rate r(30);
  while (rclcpp::ok()) {
    *temp_sensor_a = distribution(generator);
    *temp_sensor_b = distribution(generator);
    *temp_sensor_c = distribution(generator);
    exec->spin_some();
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
