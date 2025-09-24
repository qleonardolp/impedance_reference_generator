# Copyright 2026 qleonardolp
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            'param_file',
            default_value='sys_id',
            description='Parameter file name, without .yaml.',
        )
    )

    pkg_share = FindPackageShare('impedance_reference_generator')
    param_file_path = PathJoinSubstitution(
        [pkg_share, 'config', [LaunchConfiguration('param_file'), '.yaml']]
    )

    reference_generator = Node(
            package='impedance_reference_generator',
            executable='kinematic_reference',
            name='kinematic_reference',
            parameters=[param_file_path],
        )

    system_identification = Node(
            package='impedance_reference_generator',
            executable='identification',
            name='identification',
            parameters=[param_file_path],
        )

    nodes = [
        reference_generator,
        system_identification,
    ]

    return LaunchDescription(arguments + nodes)
