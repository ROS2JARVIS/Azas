from glob import glob

from setuptools import find_packages, setup

package_name = "azas_motion"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/models",
            glob("../../models/azas_*.obj") + glob("../../models/azas_*.mtl"),
        ),
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
            "cup_target_move_preview_node = azas_motion.cup_target_move_preview_node:main",
            "dispenser_sequence_preview_node = azas_motion.dispenser_sequence_preview_node:main",
            "doosan_moveit_cup_target_then_shake_node = azas_motion.doosan_moveit_cup_target_then_shake_node:main",
            "doosan_moveit_grasped_tumbler_to_dispenser_node = azas_motion.doosan_moveit_grasped_tumbler_to_dispenser_node:main",
            "gear_assembly_legacy = azas_motion.gear_assembly_legacy:main",
            "m0609_shake_joint_state_node = azas_motion.m0609_shake_joint_state_node:main",
            "measured_dispenser_collision_scene_node = azas_motion.measured_dispenser_collision_scene_node:main",
            "mp_basic_legacy = azas_motion.mp_basic_legacy:main",
            "mp_waypoint_legacy = azas_motion.mp_waypoint_legacy:main",
            "mp_waypoint_pilz_legacy = azas_motion.mp_waypoint_pilz_legacy:main",
            "mp_waypoint_pilz_lin_legacy = azas_motion.mp_waypoint_pilz_lin_legacy:main",
            "pick_and_place_legacy = azas_motion.pick_and_place_legacy:main",
            "shake_visualizer_node = azas_motion.shake_visualizer_node:main",
            "side_grasp_ik_preview_node = azas_motion.side_grasp_ik_preview_node:main",
            "syrup_pump_press_legacy = azas_motion.syrup_pump_press_legacy:main",
            "target_xyz_moveit_preview_node = azas_motion.target_xyz_moveit_preview_node:main",
            "tumbler_collision_scene_node = azas_motion.tumbler_collision_scene_node:main",
            "tumbler_floor_place_node = azas_motion.tumbler_floor_place_node:main",
            "tumbler_shake_sequence_node = azas_motion.tumbler_shake_sequence_node:main",
        ],
    },
)
