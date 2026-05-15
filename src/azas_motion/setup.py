from setuptools import find_packages, setup

package_name = "azas_motion"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Azas Team",
    maintainer_email="team@example.com",
    description="MoveItPy motion and dispenser alignment skeleton for Azas.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "alignment_executor_node = azas_motion.alignment_executor_node:main",
            "collision_obstacle_legacy = azas_motion.collision_obstacle_legacy:main",
            "dispenser_sequence_preview_node = azas_motion.dispenser_sequence_preview_node:main",
            "gear_assembly_legacy = azas_motion.gear_assembly_legacy:main",
            "mp_basic_legacy = azas_motion.mp_basic_legacy:main",
            "mp_waypoint_legacy = azas_motion.mp_waypoint_legacy:main",
            "mp_waypoint_pilz_legacy = azas_motion.mp_waypoint_pilz_legacy:main",
            "mp_waypoint_pilz_lin_legacy = azas_motion.mp_waypoint_pilz_lin_legacy:main",
            "pick_and_place_legacy = azas_motion.pick_and_place_legacy:main",
            "side_grasp_ik_preview_node = azas_motion.side_grasp_ik_preview_node:main",
            "syrup_pump_press_legacy = azas_motion.syrup_pump_press_legacy:main",
        ],
    },
)
