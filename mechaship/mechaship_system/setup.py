import os
from glob import glob

from setuptools import find_packages, setup

package_name = "mechaship_system"

setup(
    name=package_name,
    version="0.0.0",
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
    maintainer="max",
    maintainer_email="techms5499@gmail.com",
    description="mechaship background package",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "service_node = mechaship_system.service_node:main",
            "actuator_enable_node = mechaship_system.actuator_enable_node:main",
        ],
    },
)
