from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'isv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 패키지 인덱스 등록
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # package.xml 설치
        ('share/' + package_name, ['package.xml']),
        
        # [수정] launch_isv 폴더 안의 .py 파일들을 share/isv/launch 폴더로 복사
        # 이 설정이 있어야 'ros2 launch isv ...' 명령어가 작동합니다.
        (os.path.join('share', package_name, 'launch'), glob('launch_isv/*.py')),
        
        # YAML 파라미터 파일 및 기타 설정 파일을 share/isv/config 디렉토리에 설치
        (os.path.join('share', package_name, 'config'), glob('launch_isv/*.yaml')),
    ],
    install_requires=['setuptools', 'flask'], # 지도 서버 기능을 위해 flask 필요
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
            # ros2 run isv <name> 으로 실행할 수 있는 노드들
            'get_gps = launch_isv.get_gps:main',
            'isv_2026 = launch_isv.isv_2026:main',
            'detection_test = launch_isv.detection_test:main',
            'lidar_preprocess = launch_isv.Lidar_Preprocess:main',
            'obstacle_isv = launch_isv.obstacle_isv:main',
            'status3_test = launch_isv.status3_test:main',
            'gps_pursue = launch_isv.gps_pursue:main',
        ],
    },
)