from glob import glob
import os

from setuptools import find_packages, setup


package_name = "xycar_parking_nav"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob("behavior_trees/*.xml"),
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
        (os.path.join("share", package_name, "docs"), glob("docs/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team K.A.I.",
    maintainer_email="team.kai@example.com",
    description=(
        "Static-map LiDAR localization, Ackermann parking mission control, "
        "and fail-closed Xycar command adaptation."
    ),
    license="UNLICENSED",
    entry_points={
        "console_scripts": [
            "cmd_vel_adapter = xycar_parking_nav.cmd_vel_adapter:main",
            "mission_manager = xycar_parking_nav.mission_manager:main",
            "parking_preflight = xycar_parking_nav.parking_preflight:main",
            "parking_simulator = xycar_parking_nav.parking_simulator:main",
            "scan_filter = xycar_parking_nav.scan_filter_node:main",
            "vesc_imu_odom = xycar_parking_nav.vesc_imu_odom_node:main",
            "validate_map = xycar_parking_nav.validate_map:main",
        ],
    },
)
