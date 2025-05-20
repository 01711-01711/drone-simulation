from setuptools import setup

package_name = 'drone_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mateusz Zawila',
    maintainer_email='mateuszzawila01@egmail.com',
    description='Drone URDF, xacro files, and mesh assets for ROS2 simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
