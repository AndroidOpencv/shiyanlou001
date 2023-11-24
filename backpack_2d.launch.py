"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os
## ***** File paths wgb ******
def generate_launch_description():

    ## ***** File paths ******
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'backpack_2d.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            ],
        output = 'screen'
        )
    joint_state_publisher_node = Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            ],
        output = 'screen'
        )
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'backpack_2d.lua'],
        remappings = [
            ('echoes', 'horizontal_laser_2d')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'resolution': 0.05}
            ],
        )
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/2d.rviz'],
       
    )
    return LaunchDescription([
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node 
    ])
