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
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("follow_me")
    launch_dir = os.path.join(pkg, "launch")
    locator_cfg = os.path.join(pkg, "config", "locator.yaml")

    follow_distance = LaunchConfiguration("follow_distance")
    name_space = LaunchConfiguration("name_space")

    # cube.launch uses a single UWB TWR tag (C694) mounted on the AoA cube
    twr_shared = {
        "twr_ids": ["C694"],
        "twr_positions": [0.0, 0.085, -0.01],
    }

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("follow_distance", default_value="1.5"))
    ld.add_action(DeclareLaunchArgument("name_space", default_value="follow_me"))
    ld.add_action(DeclareLaunchArgument("mount_height", default_value="0.0"))

    ld.add_action(
        Node(
            package="follow_me",
            executable="publish_tfs",
            name="publish_static_tfs",
            output="screen",
        )
    )

    # uwb/twr namespace: single C694 tag driver (external dwm1001_ros)
    # ------------------------------------------------------------------
    # The ROS 1 launch started one dwm1001_ros/tag_node with
    #   usb_port=/dev/ttyACM_uwbc694
    # Uncomment and adapt once the ROS 2 driver is available, e.g.:
    #
    # ld.add_action(GroupAction(actions=[
    #     PushRosNamespace("uwb/twr"),
    #     IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory("dwm1001_ros"),
    #         "launch", "tag_node.launch.py")),
    #         launch_arguments={"usb_port": "/dev/ttyACM_uwbc694"}.items())]))
    # ------------------------------------------------------------------

    # bluetooth/aoa namespace: AoA drivers + locator
    ld.add_action(
        GroupAction(
            actions=[
                PushRosNamespace("bluetooth/aoa"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, "aoa.launch.py")
                    )
                ),
            ]
        )
    )

    # follow_me namespace: the pure pursuit velocity controller
    ld.add_action(
        GroupAction(
            actions=[
                PushRosNamespace(name_space),
                Node(
                    package="follow_me",
                    executable="velocity_ctrl",
                    name="pure_pursuit",
                    output="screen",
                    parameters=[
                        locator_cfg,
                        twr_shared,
                        {
                            "follow_distance": ParameterValue(
                                follow_distance, value_type=float
                            )
                        },
                    ],
                ),
            ]
        )
    )

    # cube namespace: cube locator + cube mesh marker
    ld.add_action(
        GroupAction(
            actions=[
                PushRosNamespace("cube"),
                Node(
                    package="follow_me",
                    executable="cube_locator",
                    name="cube_locator",
                    output="screen",
                ),
                Node(
                    package="follow_me",
                    executable="cube_marker",
                    name="cube_marker",
                    output="screen",
                ),
            ]
        )
    )

    return ld
