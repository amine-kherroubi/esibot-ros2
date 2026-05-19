from setuptools import setup
import os
from glob import glob

package_name = 'web_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EsiBot Team',
    maintainer_email='esibot@esi.dz',
    description='Passerelle WebSocket ROS2 ↔ navigateur web (rosbridge_suite)',
    license='Apache-2.0',
    entry_points={},
)
