import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mechaship_teleop"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Max Cha",
    author_email="max@mechasolution.com",
    maintainer="Mechasolution",
    maintainer_email="techms5499@gmail.com",
    description="ROS 2 package for MechaShip teleoperation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mechaship_teleop_keyboard = mechaship_teleop.mechaship_teleop_keyboard:main",
            "mechaship_teleop_joystick = mechaship_teleop.mechaship_teleop_joystick:main",
        ],
    },
)
