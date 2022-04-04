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
import launch_ros


def generate_launch_description():
    argument_node_count = launch.actions.DeclareLaunchArgument(
        'node_count', default_value='1')
    argument_executor_type = launch.actions.DeclareLaunchArgument(
        'executor_type', default_value='s')
    exec__multi_node = launch_ros.actions.Node(
        package='ch2_node_cpp',
        executable='multinode',
        arguments=[
            launch.substitutions.LaunchConfiguration('node_count'),
            launch.substitutions.LaunchConfiguration('executor_type')],
        output='screen')

    return launch.LaunchDescription([
        argument_node_count,
        argument_executor_type,
        exec__multi_node,
    ])
