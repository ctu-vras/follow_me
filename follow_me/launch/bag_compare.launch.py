"""Run the pre-refactor Cartesian-Kalman radio_locator (radio_locator_old),
and optionally a fresh instance of the new polar-Kalman radio_locator,
alongside `ros2 bag play` of a recording, so trajectories can be compared
(e.g. in rviz, or by subscribing to the estimate_pose topics).

    ros2 bag play <bag> &
    ros2 launch follow_me bag_compare.launch.py
    ros2 launch follow_me bag_compare.launch.py run_new_locator:=true

radio_locator_old always runs, pushed under the "radio_old" namespace with
human_frame "radio_position_old". run_new_locator (default false) adds a
second, fresh instance of the current radio_locator under "radio_new" /
"radio_position_new". Both are kept off the bag's own "radio" namespace /
"radio_position" frame on purpose, in case the bag already has a recorded run
of the new locator you want to compare against instead of replaying it.

IMPORTANT - verify your bag actually contains what these nodes need before
relying on their output; nothing here can be fixed by remapping a topic that
was never recorded:
  - /uwb/twr/ID_<id>/distances (dwm1001_ros_interfaces/UWBMeas) per tag in
    twr.yaml's `ids` - the primary input for BOTH locators. If it's not in
    the bag (`ros2 bag info <bag>`), neither has anything to trilaterate
    from and both just log "missing data" for the entire playback.
  - /bluetooth/aoa/angle (follow_me_interfaces/HeadingEstimate), only if
    use_aoa is left true. Note this is the aoa_locator's *output* topic, not
    its debug markers (.../heading, .../measured_angles) - those don't carry
    the actual angle estimate.
  - /tf_static must include the base_link -> aoa -> aoa1 (etc.) chain used
    by publish_tfs.py / aoa_locator.py at record time, for the AoA angle to
    be rotated into fixed_frame. `ros2 bag play` republishes /tf_static as
    transient-local, so a late-joining node (like this launch) still gets it
    as long as the bag recorded it in the first place.

The new locator additionally wants /imu/data - if your bag instead has the
IMU under a different topic (e.g. /odin1/imu), the remap below points it
there; edit it if your topic name differs. radio_locator_old doesn't use
IMU at all, so that gap doesn't apply to it.

Not needed here: publish_tfs (the bag's own /tf_static already covers it) and
use_sim_time (default `ros2 bag play` runs in real time, so wall-clock
staleness checks line up with message receipt time as usual). If you play
the bag with --clock or a rate other than 1.0, add {"use_sim_time": True} to
the node parameters below and launch with `ros2 launch ... use_sim_time:=true`,
and pass `--clock` to `ros2 bag play`.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue


def load_twr_shared(pkg):
    """mirrors radio.launch.py's loader: read the real /uwb/twr parameters
    (ids/positions/calibration/target) that radio_locator{,_old} need as
    twr_* parameters"""
    cfg = os.path.join(pkg, "config", "twr.yaml")
    with open(cfg) as f:
        params = yaml.safe_load(f)["/**"]["ros__parameters"]
    return {
        "twr_ids": params["ids"],
        "twr_positions": params["positions"],
        "twr_calibration": params["calibration"],
        "twr_target": params["target"],
    }


def generate_launch_description():
    pkg = get_package_share_directory("follow_me")
    radio_old_cfg = os.path.join(pkg, "config", "radio_old.yaml")
    radio_cfg = os.path.join(pkg, "config", "radio.yaml")

    use_aoa = LaunchConfiguration("use_aoa")
    run_new_locator = LaunchConfiguration("run_new_locator")

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "use_aoa",
            default_value="true",
            description="fold the AoA angle into both locators, same as the "
            "pre-refactor radio.yaml default for a real run",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "run_new_locator",
            default_value="false",
            description="also launch a fresh instance of the current (polar "
            "Kalman) radio_locator under radio_new - off by default since "
            "the bag usually already has its recorded output under /radio",
        )
    )

    twr_shared = load_twr_shared(pkg)

    ld.add_action(
        GroupAction(
            actions=[
                PushRosNamespace("radio_old"),
                Node(
                    package="follow_me",
                    executable="radio_locator_old",
                    name="twr_locator_old",
                    output="screen",
                    parameters=[
                        radio_old_cfg,
                        twr_shared,
                        {"use_aoa": ParameterValue(use_aoa, value_type=bool)},
                    ],
                    remappings=[
                        # keep these off the bag's own /detection_ready,
                        # /log_sound in case a live new-locator run shares
                        # the domain at the same time
                        ("/detection_ready", "/radio_old/detection_ready"),
                        ("/log_sound", "/radio_old/log_sound"),
                    ],
                ),
            ]
        )
    )

    ld.add_action(
        GroupAction(
            condition=IfCondition(run_new_locator),
            actions=[
                PushRosNamespace("radio_new"),
                Node(
                    package="follow_me",
                    executable="radio_locator",
                    name="twr_locator_new",
                    output="screen",
                    parameters=[
                        radio_cfg,
                        twr_shared,
                        {
                            "use_aoa": ParameterValue(use_aoa, value_type=bool),
                            "human_frame": "radio_position_new",
                        },
                    ],
                    remappings=[
                        ("/detection_ready", "/radio_new/detection_ready"),
                        ("/log_sound", "/radio_new/log_sound"),
                        # edit the source side if your bag's IMU topic differs
                        ("/imu/data", "/odin1/imu"),
                    ],
                ),
            ],
        )
    )

    return ld
