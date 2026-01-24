from setuptools import setup
import os
from glob import glob

package_name = 'robot_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='2D Robot Navigation Simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = robot_navigator.simulator:main',
            'robot_controller = robot_navigator.robot_controller:main',
            'obstacle_avoidance = robot_navigator.obstacle_avoidance:main',
        ],
    },
)
