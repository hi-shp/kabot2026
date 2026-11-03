from setuptools import find_packages, setup

package_name = "isv_kaboat"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="donggeon",
    maintainer_email="donggeon@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
          
      
            "gps = isv_kaboat.GPS_Spoofer:main",
            "ggps = isv_kaboat.get_gps:main",
            "ggpshz = isv_kaboat.gps_hz:main",
            "fusion = isv_kaboat.fusion_check:main",
            "imu = isv_kaboat.imu_convert:main",
            "thrust=isv_kaboat.thrust_check:main",
            
        ],
    },
)
