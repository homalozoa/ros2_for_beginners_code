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
    debug_exec = launch_ros.actions.Node(
        package='ch6_debug_cpp',
        executable='tracetest',
        prefix=['xterm -e gdb -ex run --args'],
        output='screen')

    return launch.LaunchDescription([
        debug_exec,
    ])
