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
from rclpy.node import Node


class Soliloquist(Node):

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter(name='time_cycle_s', value=0.5)
        time_cycle = self.get_parameter('time_cycle_s')
        self.declare_parameter('output_str', 'hello world')
        self.output_str = self.get_parameter('output_str')
        self.timer_handler = self.create_timer(time_cycle.value, self.timer_callback)

    def timer_callback(self):
        # output_str = self.get_parameter('output_str')
        self.get_logger().info(self.output_str.value)


def main(args=None):
    rclpy.init(args=args)
    node = Soliloquist('py_soliloquist')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
