from setuptools import setup, find_packages
import os

package_name = 'checkers_bot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/checkers_bot.launch.py']),
        (os.path.join('share', package_name, 'config'),
            [
                'checkers_bot/config/board_params.yaml',
                'checkers_bot/config/robot_params.yaml',
            ]),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='MTRX5700 Group',
    maintainer_email='student@sydney.edu.au',
    description='UR5e English checkers-playing robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_manager = checkers_bot.nodes.game_manager_node:main',
            'manipulation = checkers_bot.nodes.manipulation_node:main',
            'perception = checkers_bot.nodes.perception_node:main',
            'play_checkers = play_checkers:main',
        ],
    },
)
