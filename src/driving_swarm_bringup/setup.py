import os
from glob import glob
from setuptools import setup

package_name = "driving_swarm_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (os.path.join("share", package_name, "maps"), glob("maps/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*.pgm")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (
            os.path.join("share", package_name, "worlds"),
            glob("worlds/*.world"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="traichel",
    maintainer_email="nele.traichel@ovgu.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spawn_turtlebot = driving_swarm_bringup.spawn_turtlebot:main",
            "nav2_gazebo_spawner = driving_swarm_bringup.nav2_gazebo_spawner:main",
            "watchdog = driving_swarm_bringup.watchdog:main",
            "initial_pose_pub = driving_swarm_bringup.initial_pose_pub:main",
        ]
    },
)
