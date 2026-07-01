import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


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
    config = os.path.join(
        get_package_share_directory("follow_me"), "config", "tdoa.yaml"
    )

    ld = LaunchDescription()

    # ------------------------------------------------------------------
    # External uwb_tdoa driver (provide the ROS 2 package).
    # The ROS 1 launch started uwb_tdoa/driver.py with tdoa.yaml loaded.
    # Uncomment and adapt once the ROS 2 driver exe is available, e.g.:
    #
    # ld.add_action(Node(package="uwb_tdoa", executable="driver",
    #                    name="tdoa_driver", output="screen",
    #                    parameters=[config]))
    # ------------------------------------------------------------------

    ld.add_action(
        Node(
            package="follow_me",
            executable="tdoa_locator",
            name="tdoa_locator",
            output="screen",
            parameters=[config, load_twr_shared()],
        )
    )

    return ld
