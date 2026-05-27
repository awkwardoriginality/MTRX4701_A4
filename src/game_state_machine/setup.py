from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'game_state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (
            os.path.join('share', package_name, 'glados_folders'),
            glob('glados_folders/*.wav')
        ),

        (
            os.path.join('share', package_name, 'glados_folders', 'discard'),
            glob('glados_folders/discard/*.wav')
        ),
        (
            os.path.join('share', package_name, 'glados_folders', 'single_take'),
            glob('glados_folders/single_take/*.wav')
        ),
        (
            os.path.join('share', package_name, 'glados_folders', 'multiple_take'),
            glob('glados_folders/multiple_take/*.wav')
        ),
        (
            os.path.join('share', package_name, 'glados_folders', 'robot_turn'),
            glob('glados_folders/robot_turn/*.wav')
        ),
        (
            os.path.join('share', package_name, 'glados_folders', 'human_turn'),
            glob('glados_folders/human_turn/*.wav')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eashan-garg',
    maintainer_email='gargeashan1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'game_controller = game_state_machine.game_controller:main',
            'robot_controller = game_state_machine.robot_controller_node:main',
        ],
    },
)