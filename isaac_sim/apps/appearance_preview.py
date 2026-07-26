#!/usr/bin/env python3
"""Export high-resolution, third-person Kujiale appearance previews.

This deliberately creates no ROS graph, Nav2 process, RTX LiDAR, or formal
experiment evidence.  It reuses the campaign's anonymous Session Layer so the
five rendered images use exactly the same lighting and material-colour
profiles as the benchmark, while leaving the original USD untouched.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import html
import json
import os
from pathlib import Path
import sys
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from isaac_sim.src.config import load_project_config
from isaac_sim.src.environment_selection import (
    DEFAULT_ENVIRONMENT_ROOT,
    resolve_environment_usd,
    resolve_spawn_poses_file,
    runtime_project_stage,
)
from isaac_sim.src.experiment.appearance import (
    PROFILE_IDS,
    AppearanceManager,
    load_appearance_profiles,
)
from isaac_sim.src.robot.articulation_runtime import ArticulationRuntime
from isaac_sim.src.robot.spawn_pose_manager import SpawnPoseManager, load_spawn_poses
from isaac_sim.src.stage.physics_setup import PhysicsSetup, prepare_pacing
from isaac_sim.src.stage.scene_composer import SceneComposer
from isaac_sim.src.visualization.third_person_camera import ThirdPersonCamera


DEFAULT_PROFILE_CONFIG = (
    PROJECT_ROOT / "isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml"
)
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "data/appearance_previews"
KUJIALE_ENVIRONMENT = "kujiale_0026_A_to_B_door_open.usd"
KUJIALE_SPAWN = "long_route_start_g1"


def _positive_dimension(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if not 320 <= parsed <= 3840:
        raise argparse.ArgumentTypeError("must be between 320 and 3840")
    return parsed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Export non-front-camera Kujiale appearance previews without ROS, "
            "Nav2, or formal experiment output."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "isaac_sim/configs/project.yaml",
        help="project YAML configuration",
    )
    parser.add_argument(
        "--appearance-config",
        type=Path,
        default=DEFAULT_PROFILE_CONFIG,
        help="campaign appearance-profile YAML",
    )
    parser.add_argument(
        "--profile",
        action="append",
        choices=PROFILE_IDS,
        help="one profile to export; repeat to select several (default: all five)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="new empty output directory (default: timestamped data/appearance_previews path)",
    )
    parser.add_argument("--width", type=_positive_dimension, default=1920)
    parser.add_argument("--height", type=_positive_dimension, default=1080)
    parser.add_argument(
        "--environment-root",
        type=Path,
        default=DEFAULT_ENVIRONMENT_ROOT,
        help=f"directory containing {KUJIALE_ENVIRONMENT}",
    )
    parser.add_argument(
        "--environment-usd",
        default=KUJIALE_ENVIRONMENT,
        help="Kujiale USD absolute path, root-relative path, or unique filename",
    )
    parser.add_argument(
        "--spawn-poses-file",
        type=Path,
        default=None,
        help="explicit spawn-pose YAML (default: selected scene profile)",
    )
    parser.add_argument(
        "--spawn-pose",
        default=KUJIALE_SPAWN,
        help="named robot pose for the third-person image",
    )
    return parser


def selected_profiles(requested: Sequence[str] | None) -> tuple[str, ...]:
    """Return stable, duplicate-free profile order for image filenames."""

    if not requested:
        return PROFILE_IDS
    requested_set = set(requested)
    return tuple(profile_id for profile_id in PROFILE_IDS if profile_id in requested_set)


def resolve_output_dir(value: Path | None, *, now: datetime | None = None) -> Path:
    """Return a new, empty output directory without replacing prior previews."""

    if value is None:
        timestamp = (now or datetime.now(timezone.utc)).strftime("%Y%m%dT%H%M%SZ")
        candidate = DEFAULT_OUTPUT_ROOT / f"kujiale_appearance_{timestamp}"
    else:
        candidate = value.expanduser()
    resolved = candidate.resolve()
    if resolved.exists() and any(resolved.iterdir()):
        raise ValueError(f"preview output directory must be new or empty: {resolved}")
    resolved.mkdir(parents=True, exist_ok=True)
    return resolved


def preview_html(profile_ids: Sequence[str], width: int, height: int) -> str:
    """Create a small local gallery whose full-resolution images are clickable."""

    cards = "\n".join(
        (
            '<figure><a href="{name}.png" target="_blank" rel="noopener">'
            '<img src="{name}.png" alt="{label} third-person appearance preview"></a>'
            '<figcaption><strong>{label}</strong><br>点击图片在新标签页查看原始分辨率。</figcaption></figure>'
        ).format(
            name=html.escape(profile_id), label=html.escape(profile_id)
        )
        for profile_id in profile_ids
    )
    return f"""<!doctype html>
<html lang=\"zh-CN\"><head><meta charset=\"utf-8\"><title>Kujiale 外观预览</title>
<style>body{{font-family:system-ui,sans-serif;margin:2rem;background:#f5f7fb;color:#15213a}}
h1{{margin-bottom:.25rem}}p{{max-width:72rem;line-height:1.55}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(360px,1fr));gap:1.25rem}}
figure{{background:#fff;border:1px solid #dce3ee;border-radius:14px;margin:0;padding:1rem;box-shadow:0 1px 3px #15213a12}}
img{{width:100%;height:auto;display:block;border-radius:8px}}figcaption{{padding:.8rem .15rem .1rem}}</style></head>
<body><h1>Kujiale 光照/颜色外观预览</h1>
<p>固定 G1 出生点的高分辨率第三人称跟随视角（{width}×{height}），不是小车前向 RGB-D 图。仅用于目视核验外观变化；不包含 ROS/Nav2、动态 actor 或正式实验统计证据。</p>
<div class=\"grid\">{cards}</div></body></html>"""


def _configure_kujiale_environment(args: argparse.Namespace, output_dir: Path) -> None:
    source_asset = resolve_environment_usd(args.environment_usd, args.environment_root)
    spawn_poses = resolve_spawn_poses_file(
        source_asset,
        explicit=args.spawn_poses_file,
        repository_profiles=PROJECT_ROOT / "isaac_sim/configs/environments",
    )
    runtime_dir = output_dir / ".runtime"
    os.environ["ISAAC_NAV__ENVIRONMENT__SOURCE_ASSET"] = str(source_asset)
    os.environ["ISAAC_NAV__ENVIRONMENT__PROJECT_STAGE"] = str(
        runtime_project_stage(source_asset, runtime_dir)
    )
    os.environ["ISAAC_NAV__SPAWN__POSES_FILE"] = str(spawn_poses)
    os.environ["ISAAC_NAV__SPAWN__SELECTED"] = args.spawn_pose
    os.environ["ISAAC_NAV__SIMULATION__HEADLESS"] = "true"


def _capture_rgb(render_product: object, destination: Path) -> None:
    import omni.replicator.core as rep
    from omni.replicator.core.functional import write_image

    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(render_product)
    try:
        rep.orchestrator.set_capture_on_play(False)
        # Multiple fixed subframes reduce texture/loading noise before the
        # RGB data are read; the paused preview does not advance the robot.
        rep.orchestrator.step(rt_subframes=16, delta_time=0.0, pause_timeline=True)
        write_image(path=str(destination), data=annotator.get_data())
    finally:
        annotator.detach()


def run(args: argparse.Namespace) -> int:
    output_dir = resolve_output_dir(args.output_dir)
    profiles = selected_profiles(args.profile)
    _configure_kujiale_environment(args, output_dir)
    config = load_project_config(args.config)
    config = replace(config, simulation=replace(config.simulation, headless=True))
    profile_config = args.appearance_config.expanduser().resolve()
    appearance_profiles = load_appearance_profiles(profile_config)
    for profile_id in profiles:
        appearance_profiles.require(profile_id)

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": config.simulation.renderer,
            "multi_gpu": False,
        }
    )
    appearance_manager = None
    render_product = None
    failed = False
    try:
        import carb.settings
        import omni.replicator.core as rep

        # Quality avoids a tiny, noisy image while remaining less demanding
        # than the interactive GUI's RTX sensor stack.
        carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)
        prepare_pacing(config.simulation)
        stage = SceneComposer(config).compose(save=False)
        stage.Load()
        for _ in range(4):
            app.update()
        runtime = PhysicsSetup(config.simulation).apply(stage, app)
        runtime.reset()
        robot = ArticulationRuntime(
            config.robot.articulation_root, config.robot.base_link_prim, app
        )
        robot.initialize()
        SpawnPoseManager(robot, load_spawn_poses(config.spawn.poses_file)).apply_usd_pose(
            config.spawn.selected
        )
        for _ in range(4):
            app.update()

        camera = ThirdPersonCamera(
            stage,
            config.robot.base_link_prim,
            config.third_person_camera,
            activate_viewport=False,
        )
        render_product = rep.create.render_product(
            camera.camera_path,
            resolution=(args.width, args.height),
            force_new=True,
            name="KujialeAppearanceThirdPerson",
            render_vars=["LdrColor"],
        )
        appearance_manager = AppearanceManager(stage, appearance_profiles)

        manifest: dict[str, object] = {
            "schema_version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "view": "robot-relative third-person",
            "resolution": [args.width, args.height],
            "spawn_pose": config.spawn.selected,
            "environment_usd": str(config.environment.source_asset),
            "appearance_config": str(profile_config),
            "appearance_config_sha256": appearance_profiles.sha256,
            "profiles": {},
            "formal_experiment_evidence": False,
        }
        for profile_id in profiles:
            state = appearance_manager.apply(profile_id)
            for _ in range(3):
                app.update()
            destination = output_dir / f"{profile_id}.png"
            _capture_rgb(render_product, destination)
            if not destination.is_file() or destination.stat().st_size == 0:
                raise RuntimeError(f"appearance preview was not written: {destination}")
            manifest["profiles"][profile_id] = state
            print(f"appearance preview: {profile_id} -> {destination}")

        (output_dir / "preview_manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        (output_dir / "index.html").write_text(
            preview_html(profiles, args.width, args.height), encoding="utf-8"
        )
        print(f"appearance preview gallery: {output_dir / 'index.html'}")
    except Exception:
        failed = True
        raise
    finally:
        if appearance_manager is not None:
            appearance_manager.close()
        if render_product is not None:
            destroy = getattr(render_product, "destroy", None)
            if callable(destroy):
                destroy()
        app.close(exit_code=1 if failed else 0)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
