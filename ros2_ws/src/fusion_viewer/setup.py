from setuptools import setup

package_name = 'fusion_viewer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['fusion_viewer/launch/fusion_viewer.launch.py']),
        ('share/' + package_name + '/config', ['fusion_viewer/config/calib_example.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Overlay YOLO detections and Velodyne points on an OpenCV window',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'viewer_node = fusion_viewer.viewer_node:main',
        ],
    },
)
