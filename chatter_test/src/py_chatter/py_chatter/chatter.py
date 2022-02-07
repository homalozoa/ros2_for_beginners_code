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


class Chatter(Node):

    def __init__(self, str):
        self.name = str
        super().__init__(self.name)
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        print(self.get_chatter_name())

    def get_chatter_name(self):
        return self.name


def main(args=None):
    rclpy.init(args=args, domain_id=100)
    node = Chatter("py_chatter")
    print(node.context.get_domain_id())
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
