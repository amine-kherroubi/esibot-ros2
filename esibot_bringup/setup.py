import os
from glob import glob

from setuptools import find_packages, setup
package_name = 'esibot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required by ROS 2 to find the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EsiBot Team',
    maintainer_email='team@esibot.local',
    description='EsiBot bringup - driver node for ESP32 motor + odometry bridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This is what ros2 run uses:
            # ros2 run esibot_bringup esibot_driver
            'esibot_driver = esibot_bringup.esibot_driver:main',
        ],
    },
)
