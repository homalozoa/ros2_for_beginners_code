# Copyright (c) 2022 Homalozoa
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter


class Soliloquist(Node):

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter(name='time_cycle_s', value=0.5)
        time_cycle = self.get_parameter('time_cycle_s')
        self.declare_parameter('output_str', 'hello world')
        self.output_str = self.get_parameter('output_str')
        self.timer_handler = self.create_timer(time_cycle.value, self.timer_callback)
        self.add_on_set_parameters_callback(self.param_callback)

    def timer_callback(self):
        self.get_logger().info(self.output_str.value)

    def param_callback(self, data):
        for parameter in data:
            if parameter.name == 'output_str':
                if parameter.type_ == Parameter.Type.STRING:
                    self.output_str = parameter
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = Soliloquist('py_soliloquist')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
