from setuptools import setup
import os
from glob import glob

package_name = "land_station"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hiromichi123",
    maintainer_email="2271612727@qq.com",
    description="Pygame ground station UI for 2026 cooperative missions",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ground_station_ui = land_station.ground_station_ui:main",
        ],
    },
)
