import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cocktail_robot_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssu',
    maintainer_email='ssu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = cocktail_robot_system.vision_node:main',
            'detection_3d_node = cocktail_robot_system.detection_3d_node:main',
            'robot_move_test = cocktail_robot_system.robot_move_test:main',
            'lid_close_state_machine = cocktail_robot_system.lid_close_state_machine:main',
            'lid_pick_place_keyboard = cocktail_robot_system.lid_pick_place_keyboard:main',
        ],
    },
)
