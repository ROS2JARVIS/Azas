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
            "dispenser_sequence_preview_node = azas_motion.dispenser_sequence_preview_node:main",
            "m0609_shake_joint_state_node = azas_motion.m0609_shake_joint_state_node:main",
            "shake_visualizer_node = azas_motion.shake_visualizer_node:main",
            "side_grasp_ik_preview_node = azas_motion.side_grasp_ik_preview_node:main",
            "tumbler_floor_place_node = azas_motion.tumbler_floor_place_node:main",
            "tumbler_shake_sequence_node = azas_motion.tumbler_shake_sequence_node:main",
        ],
    },
)
