import os
from glob import glob

from setuptools import find_packages, setup

package_name = "wtrtk_ros2_driver"

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
    maintainer="mechasolution",
    maintainer_email="techms5499@gmail.com",
    description="Driver for wit motion WTRTK RTK GNSS modules",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wtrtk_ros2_driver = wtrtk_ros2_driver.wtrtk_ros2_driver:main",
        ],
    },
)
