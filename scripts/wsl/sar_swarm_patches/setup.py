import os
from glob import glob

from setuptools import setup

package_name = "sar_swarm"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fleet",
    maintainer_email="joy46336@gmail.com",
    description="SAR fixed-wing swarm mission orchestration for ROS 2 + ArduPilot SITL.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vehicle_agent = sar_swarm.vehicle_agent:main",
            "swarm_coordinator = sar_swarm.swarm_coordinator:main",
            "link_layer = sar_swarm.link_layer:main",
            "eo_ir_sensor = sar_swarm.eo_ir_sensor:main",
            "gcs_bridge = sar_swarm.gcs_bridge:main",
            "gcs_monitor = sar_swarm.gcs_monitor:main",
            "master_node = sar_swarm.master_node:main",
            "wingman_node = sar_swarm.wingman_node:main",
            "backup_node = sar_swarm.backup_node:main",
            "tot_node = sar_swarm.tot_node:main",
        ],
    },
)
