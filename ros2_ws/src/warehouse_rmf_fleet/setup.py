from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "warehouse_rmf_fleet"
share = Path("share") / package_name

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (str(share), ["package.xml"]),
        (str(share / "launch"), glob("launch/*.launch.py")),
        (str(share / "config"), glob("config/*.yaml")),
        (str(share / "config" / "nav_graphs"), glob("config/nav_graphs/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project Maintainer",
    maintainer_email="ram@example.invalid",
    description="Simulation-only Open-RMF adapter for Clearpath A300 Nav2 robots",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "fleet_adapter = warehouse_rmf_fleet.fleet_adapter:main",
            "validate_assets = warehouse_rmf_fleet.validate_assets:main",
        ],
    },
)
