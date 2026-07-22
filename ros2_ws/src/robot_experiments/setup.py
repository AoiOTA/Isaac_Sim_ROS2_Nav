from glob import glob
from setuptools import find_packages, setup


package_name = "robot_experiments"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.json")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="AoiOTA",
    maintainer_email="liang_yibo@hdu.edu.cn",
    description="Reproducible Nav2 experiment orchestration, metrics, and reports.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "experiment_runner = robot_experiments.experiment_runner:main",
            "incremental_map_compare = robot_experiments.incremental_map_compare:main",
            "initial_pose_publisher = robot_experiments.initial_pose_publisher:main",
            "motion_benchmark = robot_experiments.motion_benchmark:main",
            "navigation_benchmark = robot_experiments.navigation_benchmark:main",
            "kujiale_campaign = robot_experiments.kujiale_campaign:main",
            "kujiale_reference = robot_experiments.kujiale_reference:main",
        ],
    },
)
