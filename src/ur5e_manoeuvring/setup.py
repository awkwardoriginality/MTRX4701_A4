from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5e_manoeuvring'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eashan-garg',
    maintainer_email='eashan-garg@example.com',
    description='UR5e manoeuvring and gripper launch helpers',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chessboard_marker_node = ur5e_manoeuvring.chessboard_marker_node:main',
            'bounding_box_node = ur5e_manoeuvring.bounding_box_node:main',
            'ur5e_cartesian_node = ur5e_manoeuvring.ur5e_cartesian_node:main',
            'checkerboard_pose_node = ur5e_manoeuvring.checkerboard_pose_node:main',
            'gripper_command_node = ur5e_manoeuvring.gripper_command_node:main',
        ],
    },
)