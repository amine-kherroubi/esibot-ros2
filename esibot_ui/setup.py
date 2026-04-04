from setuptools import setup, find_packages
import os
from glob import glob


package_name = 'esibot_ui'


def collect_web_files():
    """Collecte récursivement tous les fichiers du dossier web/ (dist buildé)."""
    data = []
    web_dir = 'web'
    if not os.path.isdir(web_dir):
        return data
    for dirpath, _, filenames in os.walk(web_dir):
        if not filenames:
            continue
        install_path = os.path.join('share', package_name, dirpath)
        files = [os.path.join(dirpath, f) for f in filenames]
        data.append((install_path, files))
    return data


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        *collect_web_files(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EsiBot Team',
    maintainer_email='esibot@esi.dz',
    description='Dashboard web embarqué EsiBot (ROS2 Python + React)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dashboard_node = esibot_ui.dashboard_node:main',
            'map_saver_node = esibot_ui.map_saver_node:main',
            'nav_goal_proxy = esibot_ui.nav_goal_proxy:main',
        ],
    },
)
