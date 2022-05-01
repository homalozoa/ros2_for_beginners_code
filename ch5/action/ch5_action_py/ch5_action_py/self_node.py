# Copyright (c) 2022 Homalozoa
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

from ch5_action_interfaces.action import Count

import rclpy
import rclpy.action
import rclpy.callback_groups

from rclpy.node import Node


class ActionNodePy(Node):

    def __init__(self, name):
        super().__init__(name)
        self.global_count_ = 0
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.server_ = rclpy.action.ActionServer(self, Count, 'sec_count', self.count_callback,
                                                 callback_group=self.callback_group,
                                                 cancel_callback=self.cancel_callback)
        self.client_ = rclpy.action.ActionClient(self, Count, 'sec_count')
        timer_period = 9
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        goal = Count.Goal()
        goal.goal_count = 3
        self.client_.send_goal_async(goal)
        self.get_logger().info('Send goal: ' + str(goal.goal_count))

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return rclpy.action.CancelResponse.ACCEPT

    def count_callback(self, goal_handle):
        result = Count.Result()
        feedback = Count.Feedback()
        goal = goal_handle.request.goal_count
        local_count = 0
        self.get_logger().info('Got goal: ' + str(goal))
        while (local_count < goal):
            if goal_handle.is_cancel_requested:
                break
            feedback.local_count = local_count
            goal_handle.publish_feedback(feedback)
            self.get_logger().info('Publish feedback: ' + str(local_count))
            local_count += 1
            time.sleep(1)
        goal_handle.succeed()
        self.global_count_ += local_count
        result.global_count = self.global_count_
        self.get_logger().info('Return result: ' + str(self.global_count_))
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ActionNodePy('self_action')
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
