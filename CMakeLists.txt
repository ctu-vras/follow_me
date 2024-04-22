cmake_minimum_required(VERSION 3.0.2)
project(follow_me)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ros_numpy
  rospy
  std_msgs
  tf2
  tf2_py
  tf2_ros
  message_generation
)

catkin_python_setup()

# Generate messages in the 'msg' folder
add_message_files(
 FILES
 HumanState.msg
 HeadingEstimate.msg
 PositionEstimate.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 geometry_msgs  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES follow_me
#  CATKIN_DEPENDS geometry_msgs ros_numpy rospy std_msgs tf2 tf2_py tf2_ros
#  DEPENDS system_lib
)

# FIXME: keep the list updated with new files and names
catkin_install_python(PROGRAMS
    scripts/twr_visualizer.py
    scripts/locator_isolated.py
    scripts/velocity_ctrl.py
    scripts/tdoa_locator.py
    scripts/twr_locator.py
    scripts/aoa_locator.py
    scripts/radio_locator.py
    scripts/cube_locator.py
    scripts/cube_marker.py
    scripts/logger.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
