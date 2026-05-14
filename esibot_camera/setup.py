from setuptools import find_packages, setup

setup(
    name='esibot_camera',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/esibot_camera']),
        ('share/esibot_camera', ['package.xml']),
        ('share/esibot_camera/launch', ['launch/camera.launch.py']),
        ('share/esibot_camera/config', ['config/camera_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Idriss Yacine Ziadi',
    maintainer_email='mi_ziadi@esi.dz',
    description='ESP32-CAM video stream to ROS 2 (Jazzy)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_stream_node = esibot_camera.camera_stream_node:main',
        ],
    },
)
