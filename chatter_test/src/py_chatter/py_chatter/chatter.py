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

import os
import rclpy
import threading
from rclpy.node import Node


class Chatter(Node):

    def __init__(self, name):
        super().__init__(name)
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(
            'pid is '
            + str(os.getpid())
            + ' thread is is '
            + str(threading.current_thread().ident))


def main(args=None):
    rclpy.init(args=args)
    node = Chatter('py_chatter')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
