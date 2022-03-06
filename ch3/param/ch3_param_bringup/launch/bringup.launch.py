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

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('ch3_param_bringup')
    
    ld = launch.LaunchDescription()

    set_parameter_cmd = launch.actions.DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'param',
            'default_param.yaml'),
        description='Path to paramaters YAML file')
    set_namespace_cmd = launch.actions.DeclareLaunchArgument(
        'namespace_ext', default_value='')
    params_file = launch.substitutions.LaunchConfiguration('params_file')
    namespace_ext = launch.substitutions.LaunchConfiguration('namespace_ext')
    ld.add_action(set_parameter_cmd)
    ld.add_action(set_namespace_cmd)
    ld.add_action(launch_ros.actions.Node(
        package='ch3_param_py',
        executable='soliloquist',
        namespace=namespace_ext,
        parameters=[params_file],
        output='screen'))

    return ld
