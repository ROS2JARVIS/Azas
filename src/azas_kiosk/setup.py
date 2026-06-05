from glob import glob

from setuptools import find_packages, setup


package_name = "azas_kiosk"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Azas Team",
    maintainer_email="team@example.com",
    description="Local kiosk UI bridge for symbolic Azas cocktail ordering.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "kiosk_node = azas_kiosk.kiosk_node:main",
        ],
    },
)
