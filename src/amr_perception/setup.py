from glob import glob
import os

from setuptools import find_packages, setup

package_name = "amr_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CGainsWorks",
    maintainer_email="114424105+CGainsWorks@users.noreply.github.com",
    description="Warehouse LiDAR segmentation and work-zone occupancy.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "segmentation_node = amr_perception.nodes.segmentation_node:main",
            "zone_occupancy_node = amr_perception.nodes.zone_occupancy_node:main",
        ],
    },
)
