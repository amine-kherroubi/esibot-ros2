# setup.py
from setuptools import find_packages, setup
import os
from glob import glob

package_name = "esibot_sensors"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="amira",
    maintainer_email="amira@todo.todo",
    description="EsiBot ultrasonic radar node — servo sweep + HC-SR04 LaserScan publisher",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "radar_node = esibot_sensors.radar_node:main",
        ],
    },
)
