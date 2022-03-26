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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='CompositionDemo',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ch2_component',
                    plugin='ros_beginner::Component1',
                    name='c1'),
                ComposableNode(
                    package='ch2_component',
                    plugin='ros_beginner::Component2',
                    name='c2'),
                ComposableNode(
                    package='ch2_component',
                    plugin='ros_beginner::Component3',
                    name='c3')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
