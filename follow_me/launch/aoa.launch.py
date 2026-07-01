import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("follow_me"), "config", "aoa.yaml"
    )

    ld = LaunchDescription()

    # ------------------------------------------------------------------
    # External xplraoa_ros AoA anchor drivers (provide the ROS 2 package).
    # The ROS 1 launch started 4 anchor_node instances:
    #   usb_port=/dev/ttyUSB_aoa1..4, n_avg=3, id=an1..4
    # Uncomment and adapt once the ROS 2 driver launch/exe is available, e.g.:
    #
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    for i in range(0, 1):
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("xplraoa_ros"),
                "launch", "anchor_node.launch.py")),
            launch_arguments={
                "usb_port": f"/dev/ttyUSB{i}",
                "n_avg": "3",
                "id": f"an{i}",
            }.items()))
    # ------------------------------------------------------------------

    # the aoa.yaml parameters were loaded globally in ROS 1; here they are
    # attached directly to the aoa_locator node.
    ld.add_action(
        Node(
            package="follow_me",
            executable="aoa_locator",
            name="aoa_locator",
            output="screen",
            parameters=[config],
        )
    )

    return ld
