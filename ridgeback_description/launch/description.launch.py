# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    this_share_directory = get_package_share_directory('ridgeback_description')
    ridgeback_control_directory = get_package_share_directory('ridgeback_control')
    xacro_path = os.path.join(get_package_share_directory('ridgeback_description'), 'urdf', 'ridgeback.urdf.xacro')

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = LaunchConfiguration('config')
    physical_robot = LaunchConfiguration('physical_robot')
    robot_ip = LaunchConfiguration('robot_ip')
    control_yaml_file = LaunchConfiguration('control_yaml_file')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_config_cmd = DeclareLaunchArgument(
        'config',
        default_value=os.getenv('RIDGEBACK_CONFIG', 'base'),
        description='get the ridgeback configuration')

    declare_physical_robot_cmd = DeclareLaunchArgument(
        'physical_robot',
        default_value='false',
        description='Whether or not you run it on the physical robot')

    declare_control_yaml_cmd = DeclareLaunchArgument(
        'control_yaml_file',
        default_value=ridgeback_control_directory+'/config/control.yaml',
        description='The config file for gazebo_ros2_control, only needed in simulation')

    declare_robot_ip_cmd = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.131.1',
        description='IP address for the ridgeback in XXX.XXX.XXX.XXX format')

    robot_description = Command([
        os.path.join(this_share_directory, 'env_run'),
        ' ',
        os.path.join(this_share_directory, 'urdf', 'configs', 'base'),
        ' ',
        'xacro', ' ', xacro_path, ' ', 'physical_robot:=', physical_robot, ' ', 'control_yaml_file:=', control_yaml_file
    ])

#  robot_description_parameter = {"robot_description": Command(['xacro',' ', xacro_path, ' ', 'physical_robot:=', physical_robot])}

    # Specify the actions
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_cmd)
    ld.add_action(declare_physical_robot_cmd)
    ld.add_action(declare_robot_ip_cmd)
    ld.add_action(declare_control_yaml_cmd)

    # Add any conditioned actions
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
