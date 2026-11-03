import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mechaship_yolov8'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "param"),
            glob(os.path.join("param", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "model"),
            glob(os.path.join("model/*")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author="Sydney Kim",
    author_email="sydney@mechasolution.com",
    maintainer="Mechasolution",
    maintainer_email="techms5499@gmail.com",
    description='ROS 2 Package for YOLOv8 Detection and Visualization',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "detect_node = mechaship_yolov8.detect_node:main",
            "visualize_node = mechaship_yolov8.visualize_node:main",
        ],
    },
)
