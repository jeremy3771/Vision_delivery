#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('delivery_motion'),
            'param',
            'delivery_bot.yaml'
        )
    )

    zlac_node = Node(
        package='delivery_motion',
        executable='run_zlac',
        parameters=[param_dir],
        arguments=[],
        output="screen",
    )

    opencm_node = Node(
        package='delivery_motion',
        executable='run_cm',
        parameters=[param_dir],
        arguments=[],
        output="screen",
    )

    vel_smoother_node = Node(
        package='delivery_motion',
        executable='velocity_smoother',
        parameters=[param_dir],
        arguments=[],
        output="screen",
    )

    # create and return launch description object
    return LaunchDescription(
        [
            zlac_node,
            opencm_node,
            vel_smoother_node,
        ]
    )