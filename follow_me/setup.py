import os
from glob import glob
from setuptools import find_packages, setup

package_name = "follow_me"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Adam Herold",
    maintainer_email="herold.adam7@gmail.com",
    description="ROS 2 package for radio based follow me function",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aoa_locator = follow_me.aoa_locator:main",
            "cube_locator = follow_me.cube_locator:main",
            "cube_marker = follow_me.cube_marker:main",
            "logger = follow_me.logger:main",
            "pose_tracker = follow_me.pose_tracker:main",
            "publish_tfs = follow_me.publish_tfs:main",
            "radio_locator = follow_me.radio_locator:main",
            "radio_locator_old = follow_me.radio_locator_old:main",
            "sim_data_generator = follow_me.sim_data_generator:main",
            "tdoa_locator = follow_me.tdoa_locator:main",
            "twr_locator = follow_me.twr_locator:main",
            "twr_visualizer = follow_me.twr_visualizer:main",
            "velocity_ctrl = follow_me.velocity_ctrl:main",
        ],
    },
)
