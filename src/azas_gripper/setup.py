from setuptools import find_packages, setup

package_name = "azas_gripper"

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
    description="RG2 gripper service boundary for Azas.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rg2_gripper_node = azas_gripper.rg2_gripper_node:main",
        ],
    },
)

