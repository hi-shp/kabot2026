from setuptools import find_packages, setup

package_name = "mechaship_example"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="max",
    maintainer_email="max@nizcorp.kr",
    description="mechaship example",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "actuator_key_example = mechaship_example.actuator_key_example:main",
            "actuator_thruster_example = mechaship_example.actuator_thruster_example:main",
            "led_example = mechaship_example.led_example:main",
            "rgbw_led_example = mechaship_example.rgbw_led_example:main",
            "tone_example = mechaship_example.tone_example:main",
            "battery_example = mechaship_example.battery_example:main",
            "emo_switch_example = mechaship_example.emo_switch_example:main",
            "imu_key_example = mechaship_example.imu_key_example:main",
            "lidar_slice_front_example = mechaship_example.lidar_slice_front_example:main",
            "gps_example = mechaship_example.gps_example:main",
        ],
    },
)
