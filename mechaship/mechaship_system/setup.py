import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mechaship_system"

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
        (
            os.path.join("share", package_name, "param"),
            glob(os.path.join("param", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Max Cha",
    author_email="max@mechasolution.com",
    maintainer="Mechasolution",
    maintainer_email="techms5499@gmail.com",
    description="MechaShip background package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "actuator_enable_node = mechaship_system.actuator_enable_node:main",
        ],
    },
)
