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
import rclpy.executors

from rclpy.node import Node
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import GetParameters


class ServerNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.serv_ = self.create_service(GetParameters, 'get_para', self.srv_callback,
                                         callback_group=self.callback_group)

    def srv_callback(self, request, response):
        para_ = Parameter()
        para_.value.bool_value = bool(request.names)
        self.get_logger().info('Request: ' + request.names[0])
        response.values.append(para_.value)
        return response


class ClientNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.client_ = self.create_client(GetParameters, 'get_para')
        self.create_timer(0.5, self.clientimer_callback, self.callback_group)

    def clientimer_callback(self):
        request_ = GetParameters.Request()
        now_sec, _ = self.get_clock().now().seconds_nanoseconds()
        self.get_logger().info('Ready to send req')
        request_.names.append(str(now_sec))
        future_ = self.client_.call(request_)
        self.get_logger().info('Sent req')

        response_value_ = future_.values[0]
        if bool(response_value_.bool_value):
            self.get_logger().info('Got response succeed')


def main(args=None):
    rclpy.init(args=args)
    srv_node = ServerNode('service_server')
    cli_node = ClientNode('service_client')
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(srv_node)
    executor.add_node(cli_node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
