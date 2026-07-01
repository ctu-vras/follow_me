import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg = get_package_share_directory("follow_me")
    launch_dir = os.path.join(pkg, "launch")

    name_space = LaunchConfiguration("name_space")
    mount_height = LaunchConfiguration("mount_height")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("follow_distance", default_value="1.5"))
    ld.add_action(DeclareLaunchArgument("name_space", default_value="follow_me"))
    ld.add_action(DeclareLaunchArgument("tdoa_available", default_value="false"))
    ld.add_action(DeclareLaunchArgument("twr_available", default_value="true"))
    ld.add_action(DeclareLaunchArgument("bt_aoa_available", default_value="true"))
    ld.add_action(DeclareLaunchArgument("mount_height", default_value="0.0"))

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "sensors.launch.py")
            ),
            launch_arguments={
                "tdoa_available": LaunchConfiguration("tdoa_available"),
                "twr_available": LaunchConfiguration("twr_available"),
                "bt_aoa_available": LaunchConfiguration("bt_aoa_available"),
                "mount_height": mount_height,
            }.items(),
        )
    )

    ld.add_action(
        GroupAction(
            actions=[
                PushRosNamespace(name_space),
                Node(
                    package="follow_me",
                    executable="logger",
                    name="logger",
                    output="screen",
                ),
            ]
        )
    )

    return ld
