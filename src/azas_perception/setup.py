from glob import glob
from setuptools import find_packages, setup

package_name = "azas_perception"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.npy")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Azas Team",
    maintainer_email="team@example.com",
    description="Depth projection and tumbler pose skeleton for Azas.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bar_detect_test_legacy_node = azas_perception.bar_detect_test_legacy_node:main",
            "bar_sort_legacy_node = azas_perception.bar_sort_legacy_node:main",
            "click_pick_legacy_node = azas_perception.click_pick_legacy_node:main",
            "cup_detection_pose_bridge_node = azas_perception.cup_detection_pose_bridge_node:main",
            "gpd_grasp_adapter_node = azas_perception.gpd_grasp_adapter_node:main",
            "realsense_data_collector_legacy_node = azas_perception.realsense_data_collector_legacy_node:main",
            "simulated_cup_detection_node = azas_perception.simulated_cup_detection_node:main",
            "yolo_cup_pick_legacy_node = azas_perception.yolo_cup_pick_legacy_node:main",
            "yolo_tumbler_detector_node = azas_perception.yolo_tumbler_detector_node:main",
        ],
    },
)
