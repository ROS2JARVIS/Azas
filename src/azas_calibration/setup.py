from setuptools import find_packages, setup

package_name = "azas_calibration"

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
    description="Calibration loaders and services for Azas MVP-1.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "calibration_loader_node = azas_calibration.calibration_loader_node:main",
            "calibration_test_legacy = azas_calibration.calibration_test_legacy:main",
            "data_recording_legacy = azas_calibration.data_recording_legacy:main",
            "eye2hand_calibration_legacy = azas_calibration.eye2hand_calibration_legacy:main",
            "handeye_calibration_legacy = azas_calibration.handeye_calibration_legacy:main",
        ],
    },
)
