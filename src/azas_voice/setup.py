from glob import glob
from setuptools import find_packages, setup

package_name = "azas_voice"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Azas Team",
    maintainer_email="team@example.com",
    description="STT and symbolic cocktail recipe mapping for Azas.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "llm_recipe_mapper_node = azas_voice.llm_recipe_mapper_node:main",
            "recipe_mapper_node = azas_voice.recipe_mapper_node:main",
            "stt_node = azas_voice.stt_node:main",
            "stt_pick_and_place_legacy = azas_voice.stt_pick_and_place_legacy:main",
            "stt_robot_control_legacy = azas_voice.stt_robot_control_legacy:main",
        ],
    },
)
