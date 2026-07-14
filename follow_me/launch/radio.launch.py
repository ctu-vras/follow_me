import os

import yaml
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


def load_twr_shared():
    """read the /uwb/twr parameters that other nodes need as twr_* parameters"""
    cfg = os.path.join(
        get_package_share_directory("follow_me"), "config", "twr.yaml"
    )
    with open(cfg) as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]
    return {
        "twr_ids": params["ids"],
        "twr_positions": params["positions"],
        "twr_calibration": params["calibration"],
        "twr_target": params["target"],
        "twr_human_frame": params["human_frame"],
    }


def generate_launch_description():
    pkg = get_package_share_directory("follow_me")
    launch_dir = os.path.join(pkg, "launch")
    locator_cfg = os.path.join(pkg, "config", "locator.yaml")
    radio_cfg = os.path.join(pkg, "config", "radio.yaml")

    follow_distance = LaunchConfiguration("follow_distance")
    name_space = LaunchConfiguration("name_space")
    mount_height = LaunchConfiguration("mount_height")
    max_msg_delay = LaunchConfiguration("max_msg_delay")

    twr_shared = load_twr_shared()

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("follow_distance", default_value="1.0"))
    ld.add_action(DeclareLaunchArgument("name_space", default_value="follow_me"))
    ld.add_action(DeclareLaunchArgument("tdoa_available", default_value="false"))
    ld.add_action(DeclareLaunchArgument("twr_available", default_value="true"))
    ld.add_action(DeclareLaunchArgument("bt_aoa_available", default_value="true"))
    ld.add_action(DeclareLaunchArgument("mount_height", default_value="0.0"))
    ld.add_action(DeclareLaunchArgument("max_msg_delay", default_value="0.3"))

    ld.add_action(
        Node(
            package="follow_me",
            executable="publish_tfs",
            name="publish_static_tfs",
            output="screen",
        )
    )

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

    # follow_me namespace: the pure pursuit velocity controller
    # ld.add_action(
    #     GroupAction(
    #         actions=[
    #             PushRosNamespace(name_space),
    #             Node(
    #                 package="follow_me",
    #                 executable="velocity_ctrl",
    #                 name="pure_pursuit",
    #                 output="screen",
    #                 parameters=[
    #                     locator_cfg,
    #                     twr_shared,
    #                     {
    #                         "follow_distance": ParameterValue(
    #                             follow_distance, value_type=float
    #                         )
    #                     },
    #                 ],
    #             ),
    #         ]
    #     )
    # )

    # radio namespace: the radio (twr + aoa fusion) locator
    ld.add_action(
        GroupAction(
            actions=[
                PushRosNamespace("radio"),
                Node(
                    package="follow_me",
                    executable="radio_locator",
                    name="twr_locator",
                    output="screen",
                    parameters=[
                        radio_cfg,
                        twr_shared,
                        {
                            "mount_height": ParameterValue(mount_height, value_type=float),
                            "max_msg_delay": ParameterValue(max_msg_delay, value_type=float),
                        },
                    ],
                ),
            ]
        )
    )

    return ld
