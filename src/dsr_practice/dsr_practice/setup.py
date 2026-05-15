from setuptools import find_packages, setup
from glob import glob

package_name = 'dsr_practice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml') + glob('config/*.npy')
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deeptree',
    maintainer_email='deeptree@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mp_basic = dsr_practice.mp_basic:main',
            'mp_waypoint = dsr_practice.mp_waypoint:main',
            'mp_waypoint_pilz = dsr_practice.mp_waypoint_pilz:main',
            'mp_waypoint_pilz_lin = dsr_practice.mp_waypoint_pilz_lin:main',
            'collision_obstacle = dsr_practice.collision_obstacle:main',
            'gripper = dsr_practice.gripper:main',
            'gear_assembly = dsr_practice.gear_assembly:main',
            'click_pick_node = dsr_practice.click_pick_node:main',
            'bar_sort_node = dsr_practice.bar_sort_node:main',
            'bar_detect_test = dsr_practice.bar_detect_test:main',
            'stt_node = dsr_practice.stt_node:main',
            'stt_robot_control = dsr_practice.stt_robot_control:main',
            'stt_pick_and_place = dsr_practice.stt_pick_and_place:main',
            'realsense_data_collector = dsr_practice.realsense_data_collector:main',
            'syrup_pump_press = dsr_practice.syrup_pump_press:main',
            'yolo_cup_pick_node = dsr_practice.yolo_cup_pick_node:main',
            'joint_state_relay = dsr_practice.joint_state_relay:main',
        ],
    },
)
