import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory("follow_me"), "launch"
    )

    tdoa_available = LaunchConfiguration("tdoa_available")
    twr_available = LaunchConfiguration("twr_available")
    bt_aoa_available = LaunchConfiguration("bt_aoa_available")
    mount_height = LaunchConfiguration("mount_height")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("tdoa_available", default_value="false"))
    ld.add_action(DeclareLaunchArgument("twr_available", default_value="true"))
    ld.add_action(DeclareLaunchArgument("bt_aoa_available", default_value="true"))
    ld.add_action(DeclareLaunchArgument("mount_height", default_value="0.0"))

    # TDoA subsystem -> namespace uwb/tdoa
    ld.add_action(
        GroupAction(
            condition=IfCondition(tdoa_available),
            actions=[
                PushRosNamespace("uwb/tdoa"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, "tdoa.launch.py")
                    )
                ),
            ],
        )
    )

    # TWR subsystem -> namespace uwb/twr
    ld.add_action(
        GroupAction(
            condition=IfCondition(twr_available),
            actions=[
                PushRosNamespace("uwb/twr"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, "twr.launch.py")
                    ),
                    launch_arguments={"mount_height": mount_height}.items(),
                ),
            ],
        )
    )

    # Bluetooth AoA subsystem -> namespace bluetooth/aoa
    ld.add_action(
        GroupAction(
            condition=IfCondition(bt_aoa_available),
            actions=[
                PushRosNamespace("bluetooth/aoa"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, "aoa.launch.py")
                    )
                ),
            ],
        )
    )

    return ld
