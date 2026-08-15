#!/usr/bin/env python3
"""Render the frozen five-waypoint task and dynamic actors on Rivermark RGB."""

from __future__ import annotations

import argparse
import ast
from collections import defaultdict
import csv
import gzip
from pathlib import Path

from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
from PIL import Image
import yaml


def _load_xy_csv(path: Path) -> tuple[list[float], list[float]]:
    with gzip.open(path, 'rt', encoding='utf-8', newline='') as stream:
        rows = csv.DictReader(stream)
        points = [(float(row['x']), float(row['y'])) for row in rows]
    return [point[0] for point in points], [point[1] for point in points]


def _load_actor_tracks(path: Path) -> dict[str, tuple[list[float], list[float]]]:
    tracks: dict[str, list[tuple[float, float]]] = defaultdict(list)
    with gzip.open(path, 'rt', encoding='utf-8', newline='') as stream:
        for row in csv.DictReader(stream):
            position = ast.literal_eval(row['position'])
            point = (float(position[0]), float(position[1]))
            if not tracks[row['id']] or point != tracks[row['id']][-1]:
                tracks[row['id']].append(point)
    return {
        actor_id: (
            [point[0] for point in points],
            [point[1] for point in points],
        )
        for actor_id, points in tracks.items()
    }


def _draw_task(
    axis: plt.Axes,
    rgb: Image.Image,
    extent: tuple[float, float, float, float],
    spawn: tuple[float, float],
    goals: list[dict[str, object]],
) -> None:
    axis.imshow(rgb, origin='upper', extent=extent)
    axis.scatter(
        [spawn[0]], [spawn[1]], marker='*', s=170, color='#fff176',
        edgecolor='#111111', linewidth=1.2, zorder=8,
    )
    for goal in goals:
        x, y = goal['position']
        axis.scatter(
            [x], [y], s=72, color='#ff7043', edgecolor='white',
            linewidth=1.5, zorder=8,
        )
        axis.annotate(
            str(goal['id']), (x, y), xytext=(6, 6),
            textcoords='offset points', color='white', fontsize=10,
            fontweight='bold', zorder=9,
            bbox={'boxstyle': 'round,pad=0.18', 'fc': '#111111', 'alpha': 0.75,
                  'ec': 'none'},
        )
    axis.set_xlim(extent[0], extent[1])
    axis.set_ylim(extent[2], extent[3])
    axis.set_aspect('equal')
    axis.set_xlabel('map x (m)')
    axis.set_ylabel('map y (m)')
    axis.grid(color='white', alpha=0.18, linewidth=0.5)


def render(root: Path, output: Path) -> None:
    data = root / 'data' / 'rivermark_demo'
    formal = (
        root / 'data' / 'experiment_runs' / 'attempt31_rivermark'
        / 'formal_20260814_v075_r30_cache'
    )
    map_config = yaml.safe_load((data / 'rivermark_selected.yaml').read_text())
    goals_config = yaml.safe_load((data / 'rivermark_demo_goals.yaml').read_text())
    spawn_config = yaml.safe_load((data / 'rivermark.spawn.yaml').read_text())
    rgb = Image.open(data / 'rivermark_selected_topdown_rgb.png').convert('RGB')

    resolution = float(map_config['resolution'])
    origin_x, origin_y, _ = map(float, map_config['origin'])
    width, height = rgb.size
    extent = (
        origin_x,
        origin_x + width * resolution,
        origin_y,
        origin_y + height * resolution,
    )
    spawn_values = spawn_config['spawn_poses']['rivermark_start']['map']['position']
    spawn = (float(spawn_values[0]), float(spawn_values[1]))
    goals = list(goals_config['route'])

    static_gt = (
        formal / 'static_off' / 'runs' / 'attempt31_rivermark_static'
        / 'run-0001-seed-9001' / 'ground_truth.csv.gz'
    )
    dynamic_run = (
        formal / 'dynamic_v2_medium' / 'runs' / 'attempt31_rivermark_dynamic'
        / 'run-0001-seed-9101'
    )
    static_x, static_y = _load_xy_csv(static_gt)
    dynamic_x, dynamic_y = _load_xy_csv(dynamic_run / 'ground_truth.csv.gz')
    actor_tracks = _load_actor_tracks(dynamic_run / 'dynamic_obstacles.csv.gz')

    fig, axes = plt.subplots(1, 2, figsize=(17, 8.8), constrained_layout=True)
    for axis in axes:
        _draw_task(axis, rgb, extent, spawn, goals)

    axes[0].plot(static_x, static_y, color='#00e5ff', linewidth=2.2, zorder=6)
    axes[0].set_title('Static: representative formal run 1 (start → G1…G5)')
    axes[0].legend(
        handles=[
            Line2D([0], [0], color='#00e5ff', lw=2.2, label='executed GT trajectory'),
            Line2D([0], [0], marker='*', color='none', markerfacecolor='#fff176',
                   markeredgecolor='#111111', markersize=12, label='start'),
            Line2D([0], [0], marker='o', color='none', markerfacecolor='#ff7043',
                   markeredgecolor='white', markersize=8, label='waypoint'),
        ],
        loc='lower left', framealpha=0.88,
    )

    axes[1].plot(dynamic_x, dynamic_y, color='#00e5ff', linewidth=2.2, zorder=6)
    actor_styles = {
        'rivermark_oncoming_cart': ('#ffeb3b', 'G2 oncoming'),
        'rivermark_crossing_cart': ('#ff4081', 'G3 crossing'),
        'rivermark_same_direction_cart': ('#7c4dff', 'G4 slow lead'),
        'rivermark_temporary_block_cart': ('#69f0ae', 'G5 temporary block'),
    }
    actor_handles: list[Line2D] = [
        Line2D([0], [0], color='#00e5ff', lw=2.2, label='robot GT trajectory')
    ]
    for actor_id, (color, label) in actor_styles.items():
        x_values, y_values = actor_tracks[actor_id]
        axes[1].plot(
            x_values, y_values, color=color, linewidth=3.0,
            marker='o', markevery=[0, -1], markersize=5, zorder=7,
        )
        actor_handles.append(Line2D([0], [0], color=color, lw=3.0, label=label))
    axes[1].set_title('Dynamic v2: representative formal run 1 and four physical actors')
    axes[1].legend(handles=actor_handles, loc='lower left', framealpha=0.88)

    fig.suptitle(
        'Attempt31 Rivermark — frozen five-waypoint task on the 0.05 m physical map',
        fontsize=15,
        fontweight='bold',
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, facecolor='white')
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--root', type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument(
        '--output', type=Path,
        default=Path('data/rivermark_demo/rivermark_five_waypoint_scenarios.png'),
    )
    args = parser.parse_args()
    output = args.output if args.output.is_absolute() else args.root / args.output
    render(args.root.resolve(), output.resolve())


if __name__ == '__main__':
    main()
