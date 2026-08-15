from pathlib import Path

import pytest
import yaml

from robot_experiments.rivermark_visual_route import load_visual_route


def test_visual_route_requires_exact_five_waypoint_order(tmp_path: Path) -> None:
    config = tmp_path / "route.yaml"
    config.write_text(
        yaml.safe_dump(
            {
                "frame_id": "map",
                "route": [
                    {
                        "id": f"G{index}",
                        "position": [float(index), float(index + 1)],
                        "yaw_deg": float(index * 10),
                    }
                    for index in range(1, 6)
                ],
            }
        ),
        encoding="utf-8",
    )
    route = load_visual_route(config)
    assert [item.goal_id for item in route] == ["G1", "G2", "G3", "G4", "G5"]
    assert route[-1].x == 5.0

    payload = yaml.safe_load(config.read_text(encoding="utf-8"))
    payload["route"] = payload["route"][:-1]
    config.write_text(yaml.safe_dump(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="exactly G1..G5"):
        load_visual_route(config)


def test_one_terminal_wrapper_enables_complete_visual_stack() -> None:
    root = Path(__file__).resolve().parents[4]
    source = (root / "scripts" / "run_rivermark_visual.sh").read_text(
        encoding="utf-8"
    )
    assert "RIVERMARK_RVIZ=1" in source
    assert 'RIVERMARK_VISUAL_ROUTE="${RIVERMARK_VISUAL_ROUTE:-1}"' in source
    assert 'module2 "${scenario}" "${profile}"' in source
    assert 'RIVERMARK_DYNAMIC_CASE="full_route_four_stage"' in source
    assert 'RIVERMARK_DYNAMIC_VARIANT="v3"' in source


def test_demo_applies_appearance_and_uses_five_waypoint_runner() -> None:
    root = Path(__file__).resolve().parents[4]
    source = (root / "scripts" / "run_rivermark_demo.sh").read_text(
        encoding="utf-8"
    )
    assert '--appearance-profile "${appearance_profile}"' in source
    assert "rivermark_visual_route" in source
    assert "visual_route_args+=(--dynamic)" in source
    assert '--dynamic-case-id "${dynamic_case}"' in source
