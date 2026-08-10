from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    map_frame = LaunchConfiguration("map_frame")
    publish_rate = LaunchConfiguration("publish_rate")
    max_points = LaunchConfiguration("max_points")
    average_poses = LaunchConfiguration("average_poses")
    tf_timeout = LaunchConfiguration("tf_timeout")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("map_frame", default_value="odin_odom"))
    ld.add_action(DeclareLaunchArgument("publish_rate", default_value="1.0"))
    ld.add_action(DeclareLaunchArgument("max_points", default_value="20"))
    ld.add_action(DeclareLaunchArgument("average_poses", default_value="true"))
    ld.add_action(DeclareLaunchArgument("tf_timeout", default_value="0.4"))

    ld.add_action(
        Node(
            package="follow_me",
            executable="pose_tracker",
            name="pose_tracker",
            output="screen",
            parameters=[
                {
                    "map_frame": map_frame,
                    "publish_rate": ParameterValue(publish_rate, value_type=float),
                    "max_points": ParameterValue(max_points, value_type=int),
                    "average_poses": ParameterValue(average_poses, value_type=bool),
                    "tf_timeout": ParameterValue(tf_timeout, value_type=float),
                }
            ],
        )
    )

    return ld
