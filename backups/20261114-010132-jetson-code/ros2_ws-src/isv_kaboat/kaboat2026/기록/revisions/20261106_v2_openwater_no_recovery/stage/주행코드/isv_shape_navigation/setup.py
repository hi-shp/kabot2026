from glob import glob
from setuptools import find_packages, setup

setup(
    name='isv_shape_navigation', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/isv_shape_navigation']),
        ('share/isv_shape_navigation', ['package.xml', 'UPSTREAM.json']),
        ('share/isv_shape_navigation/web', glob('web/*')),
        ('share/isv_shape_navigation/config', glob('config/*.yaml')),
        ('share/isv_shape_navigation/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML', 'pymap3d'], zip_safe=True,
    maintainer='ISV', maintainer_email='maintainer@example.org',
    description='Standalone shape-first SLAM navigation compatible with kabot2026',
    license='MIT',
    entry_points={'console_scripts': [
        'shape_navigation = isv_shape_navigation.node:main',
        'shape_follow = isv_shape_navigation.follow_node:main',
        'gps_dock = isv_shape_navigation.follow_node:gps_main',
        'course_mission = isv_shape_navigation.follow_node:course_main',
        'course_gui_demo = isv_shape_navigation.course_demo:main',
        'shape_record = isv_shape_navigation.record:main',
        'shape_export_images = isv_shape_navigation.export_images:main',
        'shape_demo = isv_shape_navigation.demo:main',
    ]},
)
