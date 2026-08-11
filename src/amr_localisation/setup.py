from glob import glob
import os

from setuptools import find_packages, setup

package_name = "amr_localisation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.lua")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CGainsWorks",
    maintainer_email="114424105+CGainsWorks@users.noreply.github.com",
    description="Mapping, localisation and state estimation for the warehouse AMR.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scan_odometry_node = amr_localisation.nodes.scan_odometry_node:main",
        ],
    },
)
