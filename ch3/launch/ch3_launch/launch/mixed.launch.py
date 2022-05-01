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

import os

import ament_index_python
import launch

from launch.event_handlers.on_shutdown import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = launch.LaunchDescription()

    ld.add_action(launch.actions.LogInfo(msg='Hi!'))
    node_count = launch.actions.DeclareLaunchArgument('node_count', default_value='100')
    ld.add_action(node_count)
    ld.add_action(launch.actions.DeclareLaunchArgument(
        'executor_type', default_value='s'))
    ld.add_action(launch.actions.SetLaunchConfiguration('node_count', '1'))
    ld.add_action(launch.actions.SetEnvironmentVariable('ROS_DOMAIN_ID', '100'))
    ld.add_action(launch.actions.RegisterEventHandler(
        OnShutdown(on_shutdown=[launch.actions.LogInfo(msg='ROS Apps is exiting.')])))

    ld.add_action(launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            ament_index_python.packages.get_package_share_directory('ch3_launch'), 'launch'),
         '/singlexec.launch.py'])))
    ld.add_action(launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ch3_launch'), 'launch', 'multi_pkg.launch.py']))))

    return ld
