from setuptools import find_packages, setup
import os


package_name = 'perception'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), ['config/checkers_perception.yaml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='MTRX5700 Group',
    maintainer_email='student@sydney.edu.au',
    description='Standalone perception package publishing canonical checkers board state',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'checkers_perception = perception.checkers_perception_node:main',
        ],
    },
)
