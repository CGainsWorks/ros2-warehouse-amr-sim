from glob import glob
import os

from setuptools import find_packages, setup

package_name = "amr_fleet"

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
    description="VDA 5050 v2 fleet interface over MQTT for the warehouse AMR.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vda5050_adapter = amr_fleet.nodes.vda5050_adapter:main",
        ],
    },
)
