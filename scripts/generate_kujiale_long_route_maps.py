#!/usr/bin/env python3
"""Render reproducible static and dynamic Kujiale long-route map schematics.

The drawings deliberately use the OccupancyGrid geometry, not an arbitrary
floor-plan screenshot.  This keeps every waypoint and obstacle aligned to the
``warehouse_new`` map frame used by Nav2 and the experiment runner.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw, ImageFont
import yaml


ROOT = Path(__file__).resolve().parents[1]
MAP_YAML = ROOT / "data/maps/occupancy/warehouse_new.yaml"
SPAWN_YAML = (
    ROOT / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml"
)
STATIC_SCENARIO = ROOT / "ros2_ws/src/robot_experiments/config/kujiale_static_long_range.yaml"
DYNAMIC_SCENARIO = ROOT / "ros2_ws/src/robot_experiments/config/kujiale_dynamic_long_range.yaml"
PHYSICAL_DYNAMIC = ROOT / "isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml"
CAMPAIGN = ROOT / "ros2_ws/src/robot_experiments/config/kujiale_long_range_campaign.yaml"
OUTPUT_DIR = ROOT / "docs/figures"
FONT_PATH = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")

CANVAS_WIDTH = 1800
CANVAS_HEIGHT = 2680
MARGIN_X = 160
HEADER_HEIGHT = 220
FOOTER_HEIGHT = 300

INK = "#172033"
MUTED = "#536174"
BLUE = "#2563eb"
TEAL = "#0f766e"
ORANGE = "#ea580c"
PURPLE = "#7c3aed"
PINK = "#db2777"
GREEN = "#059669"
WHITE = "#ffffff"

DYNAMIC_STYLES = {
    "local_bypass": (PURPLE, "G1→G2 横向绕行", "arm/retire: G2"),
    "g2_g3_exit": (ORANGE, "G2→G3 同向释放", "arm/retire: G3"),
    "g5_g1_crossing": (PINK, "G5→G1 门洞横穿", "arm/retire: G1"),
}


def read_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"expected mapping in {path}")
    return data


def font(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(FONT_PATH), size)


def dashed_line(
    draw: ImageDraw.ImageDraw,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    fill: str,
    width: int,
    dash: float = 18.0,
    gap: float = 12.0,
) -> None:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length == 0.0:
        return
    ux, uy = dx / length, dy / length
    distance = 0.0
    while distance < length:
        stop = min(distance + dash, length)
        draw.line(
            (
                start[0] + ux * distance,
                start[1] + uy * distance,
                start[0] + ux * stop,
                start[1] + uy * stop,
            ),
            fill=fill,
            width=width,
        )
        distance += dash + gap


def arrow(
    draw: ImageDraw.ImageDraw,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    fill: str,
    width: int,
) -> None:
    draw.line((start, end), fill=fill, width=width)
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length == 0.0:
        return
    ux, uy = dx / length, dy / length
    px, py = -uy, ux
    tip = end
    left = (tip[0] - ux * 23 + px * 12, tip[1] - uy * 23 + py * 12)
    right = (tip[0] - ux * 23 - px * 12, tip[1] - uy * 23 - py * 12)
    draw.polygon((tip, left, right), fill=fill)


def text_with_box(
    draw: ImageDraw.ImageDraw,
    xy: tuple[float, float],
    text: str,
    *,
    text_font: ImageFont.FreeTypeFont,
    fill: str = INK,
    background: str = "#fffffff0",
) -> None:
    left, top, right, bottom = draw.textbbox(xy, text, font=text_font)
    padding = 7
    draw.rounded_rectangle(
        (left - padding, top - padding, right + padding, bottom + padding),
        radius=8,
        fill=background,
    )
    draw.text(xy, text, font=text_font, fill=fill)


def route_from(scenario: dict[str, Any]) -> list[dict[str, Any]]:
    route = scenario["scenario"]["route"]
    expected = ["G2", "G3", "G4", "G5", "G1"]
    if not isinstance(route, list) or [item["id"] for item in route] != expected:
        raise ValueError("long-route scenario must contain redesigned G2, G3, G4, G5, G1 order")
    return route


def three_stage_cases(physical: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    """Return the executable relay in its arm/retire order."""
    try:
        identifiers = physical["case_sets"]["full_route_three_stage"]
        cases = physical["cases"]
    except (KeyError, TypeError) as error:
        raise ValueError("physical dynamic YAML lacks full_route_three_stage") from error
    if not isinstance(identifiers, list) or not all(isinstance(item, str) for item in identifiers):
        raise ValueError("full_route_three_stage must be a list of case IDs")
    selected = [(identifier, cases[identifier]) for identifier in identifiers]
    if set(identifier for identifier, _ in selected) != set(DYNAMIC_STYLES):
        raise ValueError("three-stage set does not match the documented interactions")
    return selected


def render(kind: str, output: Path) -> None:
    map_data = read_yaml(MAP_YAML)
    spawn_data = read_yaml(SPAWN_YAML)
    static = read_yaml(STATIC_SCENARIO)
    dynamic = read_yaml(DYNAMIC_SCENARIO)
    physical_dynamic = read_yaml(PHYSICAL_DYNAMIC)
    campaign = read_yaml(CAMPAIGN)
    static_route = route_from(static)
    dynamic_route = route_from(dynamic)
    campaign_route = campaign["route"]
    if static_route != dynamic_route:
        raise ValueError("static and dynamic route coordinates must be identical")
    if [item["id"] for item in campaign_route] != [item["id"] for item in static_route]:
        raise ValueError("campaign waypoint IDs differ from executable scenarios")

    map_path = MAP_YAML.parent / str(map_data["image"])
    occupancy = Image.open(map_path).convert("RGB")
    map_width, map_height = occupancy.size
    resolution = float(map_data["resolution"])
    origin_x, origin_y, _ = map_data["origin"]
    scale = min(
        (CANVAS_WIDTH - 2 * MARGIN_X) / map_width,
        # Keep a real footer below the map; adding it after filling all
        # remaining vertical space would clip the legend on portrait maps.
        (CANVAS_HEIGHT - HEADER_HEIGHT - FOOTER_HEIGHT - 140) / map_height,
    )
    display_width, display_height = round(map_width * scale), round(map_height * scale)
    map_left = (CANVAS_WIDTH - display_width) // 2
    map_top = HEADER_HEIGHT
    occupancy = occupancy.resize((display_width, display_height), Image.Resampling.NEAREST)

    image = Image.new("RGB", (CANVAS_WIDTH, CANVAS_HEIGHT), "#f7f9fc")
    image.paste(occupancy, (map_left, map_top))
    draw = ImageDraw.Draw(image, "RGBA")
    title = "Kujiale 全屋长距离导航｜静态场景" if kind == "static" else "Kujiale 全屋长距离导航｜动态场景"
    subtitle = "warehouse_new OccupancyGrid · map 坐标系 · S(G1) → G2 → … → G5 → G1"
    draw.rectangle((0, 0, CANVAS_WIDTH, HEADER_HEIGHT), fill="#ffffff")
    draw.text((MARGIN_X, 55), title, font=font(48), fill=INK)
    draw.text((MARGIN_X, 126), subtitle, font=font(28), fill=MUTED)
    draw.rectangle(
        (map_left - 4, map_top - 4, map_left + display_width + 4, map_top + display_height + 4),
        outline="#98a2b3",
        width=4,
    )

    def pixel(position: list[float] | tuple[float, float]) -> tuple[float, float]:
        # OccupancyGrid's (0, 0) cell is bottom-left in map coordinates, while
        # a PGM's first row is top-left.  The y flip is therefore intentional.
        x, y = float(position[0]), float(position[1])
        col = (x - float(origin_x)) / resolution
        row_from_top = map_height - 1 - (y - float(origin_y)) / resolution
        return (map_left + col * scale, map_top + row_from_top * scale)

    # One-metre grid confirms scale and makes the figure usable for setup.
    x_min, x_max = float(origin_x), float(origin_x) + map_width * resolution
    y_min, y_max = float(origin_y), float(origin_y) + map_height * resolution
    grid_font = font(18)
    for value in range(math.ceil(x_min), math.floor(x_max) + 1):
        start, end = pixel((value, y_min)), pixel((value, y_max))
        draw.line((start, end), fill="#1e293b25", width=1)
        draw.text((start[0] + 4, map_top + display_height - 28), str(value), font=grid_font, fill="#334155aa")
    for value in range(math.ceil(y_min), math.floor(y_max) + 1):
        start, end = pixel((x_min, value)), pixel((x_max, value))
        draw.line((start, end), fill="#1e293b25", width=1)
        draw.text((map_left + 6, start[1] - 24), str(value), font=grid_font, fill="#334155aa")
    draw.text((map_left + display_width - 135, map_top + display_height + 18), "x / m", font=font(22), fill=MUTED)
    draw.text((map_left - 85, map_top + 6), "y / m", font=font(22), fill=MUTED)

    spawn_name = campaign["environment"]["spawn_pose_name"]
    spawn = spawn_data["spawn_poses"][spawn_name]["map"]
    start_position = spawn["position"]
    sequence = [pixel(start_position), *[pixel(item["position"]) for item in static_route]]
    for previous, current in zip(sequence, sequence[1:]):
        dashed_line(draw, previous, current, fill=TEAL, width=5)

    # Static/dynamic overlays are intentionally distinct while all room goals
    # (G2–G5) retain identical blue circles, sizes, and labels.
    if kind == "static":
        label_offsets = {
            # Keep the provisional obstacle labels readable even while their
            # centres form a compact, intentionally editable test layout.
            "rgbd_low_box_center": (-220, -50),
            "rgbd_low_box_west": (-210, 15),
            # Keep this long label inside the right edge of the map.
            "rgbd_low_bar_east": (-365, 130),
            "rgbd_low_bar_north": (18, -54),
        }
        for obstacle in campaign["static"]["obstacles"]:
            center = obstacle["center"][:2]
            size = obstacle["size"][:2]
            center_px = pixel(center)
            half_w = float(size[0]) / resolution * scale / 2
            half_h = float(size[1]) / resolution * scale / 2
            draw.rectangle(
                (center_px[0] - half_w, center_px[1] - half_h, center_px[0] + half_w, center_px[1] + half_h),
                fill="#fb923cba",
                outline=ORANGE,
                width=5,
            )
            label_dx, label_dy = label_offsets.get(obstacle["id"], (18, -50))
            text_with_box(
                draw,
                (center_px[0] + label_dx, center_px[1] + label_dy),
                f"{obstacle['id']}\n{float(size[0]):.2f} × {float(size[1]):.2f} × {float(obstacle['size'][2]):.2f} m",
                text_font=font(20),
                fill=ORANGE,
            )
        overlay_legend = "静态障碍：四个方块和两个可手调 RGB-D 低矮长条（当前草案）"
    else:
        label_offsets = {
            "local_bypass": (-280, -112),
            "g2_g3_exit": (22, -118),
            "g5_g1_crossing": (-210, 28),
        }
        for identifier, case in three_stage_cases(physical_dynamic):
            obstacle = case["obstacle"]
            gate = case["gate"]
            color, title, goal_label = DYNAMIC_STYLES[identifier]
            start = pixel(obstacle["waypoints"][0][:2])
            end = pixel(obstacle["waypoints"][-1][:2])
            dashed_line(draw, start, end, fill=color, width=8, dash=24, gap=12)
            arrow(
                draw,
                (end[0] - (end[0] - start[0]) * 0.20, end[1] - (end[1] - start[1]) * 0.20),
                end,
                fill=color,
                width=8,
            )
            half_extent = float(obstacle["size"][0]) / resolution * scale / 2.0
            draw.rectangle(
                (start[0] - half_extent, start[1] - half_extent, start[0] + half_extent, start[1] + half_extent),
                fill=f"{color}b8",
                outline=color,
                width=5,
            )
            draw.rectangle(
                (end[0] - half_extent, end[1] - half_extent, end[0] + half_extent, end[1] + half_extent),
                outline=color,
                width=6,
            )
            if gate["axis"] != "y":
                raise ValueError(f"only y gate is currently drawable: {identifier}")
            gate_left = pixel((gate["x_range"][0], gate["threshold"]))
            gate_right = pixel((gate["x_range"][1], gate["threshold"]))
            dashed_line(draw, gate_left, gate_right, fill=color, width=5, dash=12, gap=8)
            offset = label_offsets[identifier]
            text_with_box(
                draw,
                (start[0] + offset[0], start[1] + offset[1]),
                f"{title}\n{goal_label} · {float(obstacle['speed']):.2f} m/s\n触发 y={float(gate['threshold']):.2f}",
                text_font=font(20),
                fill=color,
            )
        overlay_legend = "三色动态轨迹：实心方块为出现点、空心方块为停车点；到对应航点成功后才退役"

    # G1 is intentionally both the calibrated spawn and final return point.
    start_px = pixel(start_position)
    draw.ellipse((start_px[0] - 19, start_px[1] - 19, start_px[0] + 19, start_px[1] + 19), fill="#111827", outline=WHITE, width=5)
    text_with_box(draw, (start_px[0] + 27, start_px[1] + 17), f"S / G1\n[{start_position[0]:.2f}, {start_position[1]:.2f}]", text_font=font(22), fill="#111827")

    for waypoint in static_route:
        if waypoint["id"] == "G1":
            continue
        goal = pixel(waypoint["position"])
        radius = 18
        draw.ellipse((goal[0] - radius, goal[1] - radius, goal[0] + radius, goal[1] + radius), fill=BLUE, outline=WHITE, width=5)
        yaw = math.radians(float(waypoint["yaw_deg"]))
        arrow(
            draw,
            (goal[0], goal[1]),
            (goal[0] + math.cos(yaw) * 54, goal[1] - math.sin(yaw) * 54),
            fill=BLUE,
            width=5,
        )
        text_with_box(
            draw,
            (goal[0] + 23, goal[1] - 50),
            f"{waypoint['id']}\n[{waypoint['position'][0]:.2f}, {waypoint['position'][1]:.2f}]",
            text_font=font(22),
            fill=BLUE,
        )

    footer_top = map_top + display_height + 35
    draw.rounded_rectangle(
        (MARGIN_X, footer_top, CANVAS_WIDTH - MARGIN_X, CANVAS_HEIGHT - 65),
        radius=22,
        fill="#ffffff",
        outline="#d0d5dd",
        width=2,
    )
    draw.text((MARGIN_X + 34, footer_top + 27), "图例与使用边界", font=font(31), fill=INK)
    legend = [
        ("●", BLUE, "G2–G5：相同样式的房间航点；箭头为要求朝向"),
        ("●", "#111827", "S / G1：long_route_start_g1 与返回点，坐标重合"),
        ("– –", TEAL, "青绿虚线：航点发送顺序示意，不代表 Nav2 理论最优或实际轨迹"),
        ("■", ORANGE if kind == "static" else PURPLE, overlay_legend),
    ]
    for row, (symbol, color, description) in enumerate(legend):
        y = footer_top + 88 + row * 48
        draw.text((MARGIN_X + 42, y), symbol, font=font(30), fill=color)
        draw.text((MARGIN_X + 100, y + 3), description, font=font(23), fill=INK)
    draw.text(
        (MARGIN_X + 34, CANVAS_HEIGHT - 106),
        "来源：warehouse_new.yaml、Kujiale 长距离 static/dynamic scenario、campaign YAML 和校验过的 G1 派生出生点。",
        font=font(20),
        fill=MUTED,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, optimize=True)


def render_three_stage_details(output: Path) -> None:
    """Render the two constrained interaction zones at a readable scale."""
    map_data = read_yaml(MAP_YAML)
    physical = read_yaml(PHYSICAL_DYNAMIC)
    occupancy = Image.open(MAP_YAML.parent / str(map_data["image"])).convert("RGB")
    resolution = float(map_data["resolution"])
    origin_x, origin_y, _ = map_data["origin"]
    map_width, map_height = occupancy.size
    cases = dict(three_stage_cases(physical))

    canvas = Image.new("RGB", (1800, 1180), "#f7f9fc")
    draw = ImageDraw.Draw(canvas, "RGBA")
    draw.rectangle((0, 0, 1800, 170), fill=WHITE)
    draw.text((100, 43), "Kujiale 三阶段动态避障｜局部交互细节", font=font(46), fill=INK)
    draw.text((100, 111), "warehouse_new OccupancyGrid · 实心方块=出现点 · 空心方块=停车点 · 虚线=机器人触发门", font=font(25), fill=MUTED)

    panels = [
        ("g2_g3_exit", "G2→G3：窄通道同向释放", (-1.25, -1.05, 0.45, 2.55), (90, 230, 790, 1080)),
        ("g5_g1_crossing", "G5→G1：门洞横穿后左侧通过", (-1.40, -2.15, 0.30, -0.50), (930, 230, 1630, 1080)),
    ]
    for identifier, heading, bounds, panel in panels:
        x_min, y_min, x_max, y_max = bounds
        panel_left, panel_top, panel_right, panel_bottom = panel
        panel_width, panel_height = panel_right - panel_left, panel_bottom - panel_top
        scale = min((panel_width - 54) / (x_max - x_min), (panel_height - 132) / (y_max - y_min))
        render_width = round((x_max - x_min) * scale)
        render_height = round((y_max - y_min) * scale)
        map_left = panel_left + (panel_width - render_width) // 2
        map_top = panel_top + 86 + (panel_height - 114 - render_height) // 2
        col_left = max(0, int(math.floor((x_min - float(origin_x)) / resolution)))
        col_right = min(map_width, int(math.ceil((x_max - float(origin_x)) / resolution)))
        row_top = max(0, int(math.floor(map_height - (y_max - float(origin_y)) / resolution)))
        row_bottom = min(map_height, int(math.ceil(map_height - (y_min - float(origin_y)) / resolution)))
        crop = occupancy.crop((col_left, row_top, col_right, row_bottom)).resize(
            (render_width, render_height), Image.Resampling.NEAREST
        )
        draw.rounded_rectangle(panel, radius=24, fill=WHITE, outline="#d0d5dd", width=2)
        draw.text((panel_left + 28, panel_top + 25), heading, font=font(28), fill=INK)
        canvas.paste(crop, (map_left, map_top))
        draw.rectangle((map_left - 3, map_top - 3, map_left + render_width + 3, map_top + render_height + 3), outline="#98a2b3", width=3)

        def pixel(position: list[float] | tuple[float, float]) -> tuple[float, float]:
            return (map_left + (float(position[0]) - x_min) * scale, map_top + (y_max - float(position[1])) * scale)

        case = cases[identifier]
        obstacle = case["obstacle"]
        gate = case["gate"]
        color, _, goal_label = DYNAMIC_STYLES[identifier]
        start, end = pixel(obstacle["waypoints"][0][:2]), pixel(obstacle["waypoints"][-1][:2])
        dashed_line(draw, start, end, fill=color, width=8, dash=22, gap=12)
        arrow(draw, (end[0] - (end[0] - start[0]) * 0.20, end[1] - (end[1] - start[1]) * 0.20), end, fill=color, width=8)
        half_extent = float(obstacle["size"][0]) * scale / 2.0
        draw.rectangle((start[0] - half_extent, start[1] - half_extent, start[0] + half_extent, start[1] + half_extent), fill=f"{color}b8", outline=color, width=5)
        draw.rectangle((end[0] - half_extent, end[1] - half_extent, end[0] + half_extent, end[1] + half_extent), outline=color, width=6)
        gate_left = pixel((gate["x_range"][0], gate["threshold"]))
        gate_right = pixel((gate["x_range"][1], gate["threshold"]))
        dashed_line(draw, gate_left, gate_right, fill=color, width=5, dash=12, gap=8)
        text_with_box(
            draw,
            (panel_left + 28, panel_bottom - 72),
            f"{goal_label} · gate: y {('≤' if gate['direction'] == 'negative' else '≥')} {float(gate['threshold']):.2f} · {float(obstacle['speed']):.2f} m/s",
            text_font=font(19),
            fill=color,
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output, optimize=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    arguments = parser.parse_args()
    render("static", arguments.output_dir / "kujiale_long_route_static_map.png")
    render("dynamic", arguments.output_dir / "kujiale_long_route_dynamic_map.png")
    render_three_stage_details(arguments.output_dir / "kujiale_three_stage_dynamic_details.png")


if __name__ == "__main__":
    main()
