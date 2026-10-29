from setuptools import setup, find_packages
from glob import glob

setup(name='isv_water_trials', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/isv_water_trials']),
                  ('share/isv_water_trials', ['package.xml', 'README_KO.md', 'ANALYSIS_KO.md', 'TESTING.md']),
                  ('share/isv_water_trials/config', glob('config/*'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='ISV', maintainer_email='maintainer@example.org', license='MIT',
      entry_points={'console_scripts': [
          'record = isv_water_trials.recorder:main',
          'mark = isv_water_trials.marker:main',
          'analyze = isv_water_trials.analyze:main',
          'trial = isv_water_trials.trial:main',
          'preflight = isv_water_trials.preflight:main',
          'mpc_replay = isv_water_trials.mpc:main',
          'command_proxy = isv_water_trials.proxy:main',
          'arduino_bridge = isv_water_trials.arduino_bridge:main',
          'mode_test = isv_water_trials.mode_test:main']})
