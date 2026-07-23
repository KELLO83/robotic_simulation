"""Package definition for Isaac Stage 4 SLAM tools."""

from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "isaac_stage4_slam"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.lua")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rr",
    maintainer_email="rr@localhost.com",
    description="Cartographer and RViz mapping for the Isaac Stage 4 Waffle Pi.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wall_avoidance = isaac_stage4_slam.wall_avoidance:main",
        ],
    },
)
