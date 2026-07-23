"""Package definition for the Isaac ROS 2 compatibility facade."""

from setuptools import find_packages, setup


PACKAGE_NAME = "turtlebot3_isaac_adapter"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", ["launch/training_nodes.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rr",
    maintainer_email="rr@localhost.com",
    description="Gazebo API compatibility facade for TurtleBot3 DRL on Isaac Sim.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gazebo_api_adapter = turtlebot3_isaac_adapter.gazebo_api_adapter:main",
        ],
    },
)
