from glob import glob

from setuptools import find_packages, setup


package_name = "azas_dispenser"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/models/dispenser",
            glob("models/dispenser/*"),
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Azas Team",
    maintainer_email="team@example.com",
    description="Azas dispenser press task package for Doosan ROS2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dispenser_press_node = azas_dispenser.dispenser_press_node:main",
            "dispenser_press_moveit_node = azas_dispenser.dispenser_press_moveit_node:main",
            "find_press_ready_pose_node = azas_dispenser.find_press_ready_pose_node:main",
        ],
    },
)
