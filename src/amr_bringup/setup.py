from glob import glob
import os

from setuptools import find_packages, setup

package_name = "amr_bringup"

# The world, zones and map live in simulation/ at the repo root, which is where
# the generator writes them and where they are easiest to inspect. They are
# installed into this package's share directory so that launch files can find
# them through FindPackageShare rather than by guessing at a source path that
# does not exist in an installed workspace.
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SIMULATION = os.path.join(REPO_ROOT, "simulation")

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob(os.path.join(SIMULATION, "worlds", "*.world"))),
        (os.path.join("share", package_name, "zones"), glob(os.path.join(SIMULATION, "zones", "*.yaml"))),
        (
            os.path.join("share", package_name, "maps"),
            glob(os.path.join(SIMULATION, "maps", "*.yaml"))
            + glob(os.path.join(SIMULATION, "maps", "*.pgm")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CGainsWorks",
    maintainer_email="114424105+CGainsWorks@users.noreply.github.com",
    description="Bringup, mission execution, evaluation logging and fault injection.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_logger = amr_bringup.nodes.pose_logger:main",
            "mission_runner = amr_bringup.nodes.mission_runner:main",
            "fault_injector = amr_bringup.nodes.fault_injector:main",
        ],
    },
)
