#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare an arguments
    args = [DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Enable or disable ROS 2 bag recording'
    ),
        DeclareLaunchArgument("name", default_value="walker_node", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params",
                              default_value=os.path.join(get_package_share_directory("walker"), "config", "params.yml"),
                              description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="ROS logging level (debug, info, warn, error, fatal)"),
    ]

    # Use a function to conditionally launch the bag recording
    def launch_rosbag_record(context, *args, **kwargs):
        if LaunchConfiguration('record_bag').perform(context) == 'true':
            return [
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a'],
                    output='screen'
                )
            ]
        return []

    nodes = [
        Node(
            package="walker",
            executable="walker_node",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            output="screen",
            emulate_tty=True,
        )
    ]

    return LaunchDescription([
        *args,
        *nodes,
        OpaqueFunction(function=launch_rosbag_record)
    ])
