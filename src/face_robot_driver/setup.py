from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'face_robot_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'pyyaml', 'opencv-python', 'mediapipe'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='POM SC32 舵机驱动板 ROS 2 驱动节点',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_node      = face_robot_driver.driver_node:main',
            'eye_node         = face_robot_driver.eye_node:main',
            'face_tracker     = face_robot_driver.face_tracker_node:main',
            'emotion_mirror   = face_robot_driver.emotion_mirror_node:main',
        ],
    },
)
