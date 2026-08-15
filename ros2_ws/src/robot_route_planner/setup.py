from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robot_route_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="AoiOTA",
    maintainer_email="liang_yibo@hdu.edu.cn",
    description="Attempt30/A21 structural graph, route state, and guidance for Nav2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "structural_graph = robot_route_planner.ros_node:main",
            "build_graph = robot_route_planner.cli:main",
            "visualize_graph = robot_route_planner.visualize:main",
            "visualize_route_ab = robot_route_planner.route_ab_visualize:main",
            "probe_smac = robot_route_planner.smac_probe:main",
            "probe_execution = robot_route_planner.execution_probe:main",
            "visualize_smac = robot_route_planner.smac_visualize:main",
            "visualize_runtime = robot_route_planner.runtime_visualize:main",
            "probe_structural = robot_route_planner.structural_probe:main",
            "probe_closed_loop = robot_route_planner.closed_loop_probe:main",
            "export_v310_cognitive_pair = robot_route_planner.cognitive_pair:main",
            "build_multiroute_benchmark = robot_route_planner.benchmark_cli:main",
        ],
    },
)
