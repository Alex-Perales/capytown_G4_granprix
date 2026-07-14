from setuptools import setup
import os
from glob import glob

package_name = "capytown_granprix_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CapyTown G4",
    maintainer_email="team@capytown.local",
    description="CapyTown Gran Prix: fusión LiDAR (maze_solver, box_detector) + cámara (pare_detector) "
                "para el laberinto 'El Qhapaq Ñan de CapyTown'.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "maze_solver = capytown_granprix_pkg.maze_solver:main",
            "pare_detector = capytown_granprix_pkg.pare_detector:main",
            "box_detector = capytown_granprix_pkg.box_detector:main",
            "scan_map_viewer = capytown_granprix_pkg.scan_map_viewer:main",
            "web_dashboard = capytown_granprix_pkg.web_dashboard:main",
            "desktop_dashboard = capytown_granprix_pkg.desktop_dashboard:main",
            "simple_camera = capytown_granprix_pkg.simple_camera:main",
        ],
    },
)
