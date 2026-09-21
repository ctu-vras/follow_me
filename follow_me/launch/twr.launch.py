import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("follow_me"), "config", "twr.yaml"
    )
    mount_height = LaunchConfiguration("mount_height")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("mount_height", default_value="0.0"))

    # ------------------------------------------------------------------
    # External dwm1001_ros UWB TWR tag drivers (provide the ROS 2 package).
    # The ROS 1 launch started 3 tag_node instances:
    #   usb_port=/dev/ttyACM_uwb5722, /dev/ttyACM_uwb5a84, /dev/ttyACM_uwbc4b5
    # Uncomment and adapt once the ROS 2 driver launch/exe is available, e.g.:
    #
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    for port in ("0", "1", "2"):
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("dwm1001_ros"),
                "launch", "tag_node.launch.py")),
            launch_arguments={"usb_port": f"/dev/ttyACM{port}",
                              "tag_id": port}.items()))
    # ------------------------------------------------------------------

    mount_height_param = {
        "mount_height": ParameterValue(mount_height, value_type=float)
    }

    ld.add_action(
        Node(
            package="follow_me",
            executable="twr_visualizer",
            name="twr_visualizer",
            output="screen",
            parameters=[config],
        )
    )
    ld.add_action(
        Node(
            package="follow_me",
            executable="twr_locator",
            name="twr_locator",
            output="screen",
            parameters=[config, mount_height_param],
        )
    )

    return ld
