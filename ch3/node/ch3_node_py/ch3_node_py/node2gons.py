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


class Node2Go(Node):

    def __init__(self, name):
        self.name = name
        super().__init__(self.name)


def main(args=None):
    rclpy.init(args=args)
    node = Node2Go('py_node')
    print(node.get_namespace() + '/' + node.get_name())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
