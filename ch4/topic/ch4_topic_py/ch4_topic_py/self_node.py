# Copyright 2022 Homalozoa
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
from rclpy.qos import QoSPresetProfiles as PresetQoS
from builtin_interfaces.msg import Time


class PubNodePy(Node):

    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Time, '_current_time', 10)
        self.publisher_.get_subscription_count()
        self.subscription_ = self.create_subscription(
            Time, '_current_time', self.sub_callback, 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info(
            'pub: Current timestamp is : ' +
            str(msg.sec) +
            ' seconds, ' +
            str(msg.nanosec) +
            ' nanoseconds.')

    def sub_callback(self, msg):
        self.get_logger().info(
            'sub: Current timestamp is : ' +
            str(msg.sec) +
            ' seconds, ' +
            str(msg.nanosec) +
            ' nanoseconds.')


def main(args=None):
    rclpy.init(args=args)
    node = PubNodePy('selftalk')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
