import os
from pathlib import Path
from setuptools import find_packages, setup


package_name = "robot_experiments"
package_root = Path(__file__).resolve().parent
PACKAGED_CONFIGS = (
    "attempt31_rivermark_appearance.yaml",
    "attempt31_rivermark_dynamic.yaml",
    "attempt31_rivermark_static.yaml",
    "final_rivermark_appearance.yaml",
    "final_rivermark_dynamic.yaml",
    "final_rivermark_static.yaml",
    "kujiale_4x20_dynamic_pair.yaml",
    "kujiale_4x20_static_pair.yaml",
    "kujiale_contact_observability_dynamic.yaml",
    "kujiale_dynamic_controlled_20.yaml",
    "kujiale_dynamic_full_route_5.yaml",
    "kujiale_dynamic_visual.yaml",
    "kujiale_dynamic_visual_g2_g3.yaml",
    "kujiale_dynamic_visual_g5_g1.yaml",
    "kujiale_g2_dynamic_safety_smoke.yaml",
    "kujiale_long_range_campaign.yaml",
    "kujiale_static_long_range.yaml",
    "kujiale_static_visual.yaml",
    "module1_targeted_teaching_kujiale_en.yaml",
    "module1_targeted_teaching_kujiale_sw.yaml",
    "module1_targeted_teaching_kujiale_t1.yaml",
    "module1_targeted_teaching_kujiale_t2.yaml",
    "module1_targeted_teaching_kujiale_v1.yaml",
    "module1_targeted_teaching_kujiale_v2.yaml",
    "module1_targeted_teaching_rosbag_qos.yaml",
    "optimal_reference.json",
    "scenario.schema.yaml",
    "static.yaml",
    "v6_estimated_calibration.yaml",
    "v6_final_kujiale_appearance.yaml",
    "v6_final_kujiale_dynamic.yaml",
    "v6_final_kujiale_static.yaml",
    "v6_imu_lidar_readiness.yaml",
    "v6_imu_regime_diagnostic.yaml",
    "v6_imu_regime_resources.json",
    "v6_kujiale_low_obstacle_causal.yaml",
    "v6_kujiale_low_obstacles_static.yaml",
    "v6_localization_causal.yaml",
    "v6_low_obstacle_phase_f_rosbag_qos.yaml",
    "v6_phase_g_causal.yaml",
    "v6_pilot_kujiale_dynamic_hotreset_v1.yaml",
    "v6_pilot_kujiale_static_hotreset.yaml",
    "v6_r3_phase2_kujiale_baseline.yaml",
    "v6_r5_phase_b_kujiale_exact_baseline.yaml",
    "v6_run4_shadow_probe.yaml",
)


def source_paths(directory: str, pattern: str) -> list[str]:
    """Keep package data rooted at source even when colcon invokes setup from build/."""
    return [os.path.relpath(path, Path.cwd()) for path in sorted((package_root / directory).glob(pattern))]


def source_files(directory: str, filenames: tuple[str, ...]) -> list[str]:
    """Resolve an explicit set of package data files relative to the invoking cwd."""
    return [
        os.path.relpath(package_root / directory / filename, Path.cwd())
        for filename in filenames
    ]


def external_paths(directory: Path, pattern: str) -> list[str]:
    """Install a bounded repository resource without depending on cwd."""
    return [os.path.relpath(path, Path.cwd()) for path in sorted(directory.glob(pattern))]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            source_files("config", PACKAGED_CONFIGS)
            + external_paths(
                package_root.parents[2] / "isaac_sim/configs/experiments",
                "v6_calibration_grid_features.yaml",
            ),
        ),
        (
            f"share/{package_name}/environments",
            external_paths(
                package_root.parents[2] / "isaac_sim/configs/environments",
                "v6_calibration_flat_20m.spawn.yaml",
            ),
        ),
        (
            f"share/{package_name}/phase_f_assets/data/maps/occupancy",
            external_paths(
                package_root.parents[2] / "data/maps/occupancy",
                "v6_kujiale_isaacgen_v1.*",
            ),
        ),
        (
            f"share/{package_name}/phase_f_assets/isaac_sim/configs/environments",
            external_paths(
                package_root.parents[2] / "isaac_sim/configs/environments",
                "kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml",
            ),
        ),
        (
            f"share/{package_name}/phase_f_assets/isaac_sim/configs/experiments",
            external_paths(
                package_root.parents[2] / "isaac_sim/configs/experiments",
                "v6_kujiale_low_obstacles_frozen*.yaml",
            )
            + external_paths(
                package_root.parents[2] / "isaac_sim/configs/experiments",
                "v6_kujiale_dynamic_g2_crossing.yaml",
            ),
        ),
        (
            f"share/{package_name}/phase_f_assets/ros2_ws/src/robot_route_planner/config",
            external_paths(
                package_root.parent / "robot_route_planner/config",
                "v6_kujiale_isaacgen_v1_gvg_v1.geojson",
            ),
        ),
        (
            f"share/{package_name}/phase_f_assets/ros2_ws/src/robot_navigation/config",
            external_paths(
                package_root.parent / "robot_navigation/config",
                "nav2_v6_low_obstacle_isolation.yaml",
            ),
        ),
        (f"share/{package_name}/launch", source_paths("launch", "*.launch.py")),
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
            "outdoor_initial_pose = robot_experiments.outdoor_initial_pose:main",
            "runtime_blockage_demo = robot_experiments.runtime_blockage_demo:main",
            "motion_benchmark = robot_experiments.motion_benchmark:main",
            "navigation_benchmark = robot_experiments.navigation_benchmark:main",
            "kujiale_campaign = robot_experiments.kujiale_campaign:main",
            "dynamic_avoidance_campaign = robot_experiments.dynamic_avoidance_campaign:main",
            "kujiale_4x20_campaign = robot_experiments.kujiale_4x20_campaign:main",
            "attempt31_rivermark_qualification = robot_experiments.attempt31_rivermark_qualification:main",
            "final_rivermark_qualification = robot_experiments.final_rivermark_qualification:main",
            "final_rivermark_pilot_check = robot_experiments.final_rivermark_qualification:pilot_main",
            "kujiale_reference = robot_experiments.kujiale_reference:main",
            "rivermark_reference = robot_experiments.rivermark_reference:main",
            "rivermark_visual_route = robot_experiments.rivermark_visual_route:main",
            "estimated_state_evaluator = robot_experiments.estimated_state_evaluator:main",
            "v6_estimated_calibration = robot_experiments.v6_estimated_calibration:main",
            "v6_formal_episode = robot_experiments.v6_formal:main",
            "module1_targeted_teaching = robot_experiments.module1_targeted_teaching:main",
            "v6_low_obstacle_causal = robot_experiments.v6_low_obstacle_causal:main",
            "v6_phase_f_active_ttl_probe = robot_experiments.v6_phase_f_active_ttl_probe:main",
            "v6_localization_causal = robot_experiments.v6_localization_causal:main",
            "v6_localization_causal_evaluator = robot_experiments.v6_localization_causal_evaluator:main",
            "v6_phase_g_causal = robot_experiments.v6_phase_g_causal:main",
            "imu_regime_analysis = robot_experiments.imu_regime_analysis:main",
            "v6_imu_lidar_preflight = robot_experiments.v6_imu_lidar_preflight:main",
        ],
    },
)
