from setuptools import find_packages, setup

package_name = "azas_task_manager"

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
    description="PickAndAlign action orchestration for Azas MVP-1.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pick_and_align_action_server = azas_task_manager.pick_and_align_action_server:main",
            "cocktail_dryrun_sequence_node = azas_task_manager.cocktail_dryrun_sequence_node:main",
        ],
    },
)
