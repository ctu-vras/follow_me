import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue


def load_twr_shared(pkg):
    """read the simulated /uwb/twr parameters that other nodes need as twr_* parameters"""
    cfg = os.path.join(pkg, "config", "twr_sim.yaml")
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
    aoa_cfg = os.path.join(pkg, "config", "aoa.yaml")
    radio_cfg = os.path.join(pkg, "config", "radio.yaml")

    use_aoa = LaunchConfiguration("use_aoa")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("use_aoa", default_value="true"))

    twr_shared = load_twr_shared(pkg)

    # static tf: base_link -> locator, base_link -> aoa (needed by radio_locator / aoa_locator)
    ld.add_action(
        Node(
            package="follow_me",
            executable="publish_tfs",
            name="publish_static_tfs",
            output="screen",
        )
    )

    # artificial data generator: robot + tag ground truth trajectories, plus
    # simulated (noisy, delayed) UWB TWR and Bluetooth AoA measurements
    ld.add_action(
        Node(
            package="follow_me",
            executable="sim_data_generator",
            name="sim_data_generator",
            output="screen",
            parameters=[twr_shared, aoa_cfg],
        )
    )

    # bluetooth/aoa namespace: the AoA heading locator under test (subscribes
    # to the raw Angles topics published by sim_data_generator)
    ld.add_action(
        GroupAction(
            actions=[
                PushRosNamespace("bluetooth/aoa"),
                Node(
                    package="follow_me",
                    executable="aoa_locator",
                    name="aoa_locator",
                    output="screen",
                    parameters=[aoa_cfg],
                ),
            ]
        )
    )

    # radio namespace: the combined TWR(+AoA) locator under test
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
                        {"use_aoa": ParameterValue(use_aoa, value_type=bool)},
                    ],
                ),
            ]
        )
    )

    return ld
