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
            'pursue_lidar = launch_isv.pursue_lidar:main',
            'pursue_gps = launch_isv.pursue_gps:main',
            'course1 = launch_isv.course1:main',
            'course2 = launch_isv.course2:main',
            'course3 = launch_isv.course3:main',
        ],
    },
)