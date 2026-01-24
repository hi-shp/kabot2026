from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'isv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), glob('launch_isv/*.py')),
        (os.path.join('share', package_name, 'config'), glob('launch_isv/*.yaml')),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='GPS Logging and Real-time Leaflet Map Server',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'isv_2026 = launch_isv.isv_2026:main',
            'lidar_process = launch_isv.lidar_process:main',
            'status3_test = launch_isv.status3_test:main',
            'gps_pursue = launch_isv.gps_pursue:main',
        ],
    },
)