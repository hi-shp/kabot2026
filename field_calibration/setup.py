from glob import glob
from setuptools import find_packages, setup

setup(
    name='field_calibration', version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/field_calibration']),
        ('share/field_calibration', ['package.xml', 'README.md']),
        ('share/field_calibration/launch', glob('launch/*.launch.py')),
        ('share/field_calibration/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='hi-shp', maintainer_email='hishp@pusan.ac.kr',
    description='Passive LiDAR/IMU vessel calibration tools', license='Apache-2.0',
    entry_points={'console_scripts': [
        f'{name} = field_calibration.{name}:main' for name in (
            'field_logger', 'calibration_node', 'state_monitor',
            'motion_identifier', 'motion_predictor', 'trajectory_monitor', 'mock_sensors')
    ]},
)
