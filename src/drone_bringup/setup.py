from setuptools import setup
import os
from glob import glob

package_name = 'drone_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mateusz Zawila',
    maintainer_email='mateuszzawila01@egmail.com',
    description='ROS2 bringup package for drone with umbrella mechanism',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init_joint_positions = drone_bringup.init_joint_positions:main',
            'pid_control_node = drone_bringup.pid_control_node:main',
            'slider_control_service = drone_bringup.slider_control_service:main',
            'toggle_mechanism_service = drone_bringup.toggle_mechanism_service:main',
            'mechanism_gui = drone_bringup.mechanism_gui:main',
            'dummy_joint_state_publisher = drone_bringup.dummy_joint_state_publisher:main',
            'dummy_imu_publisher = drone_bringup.dummy_imu_publisher:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
)
