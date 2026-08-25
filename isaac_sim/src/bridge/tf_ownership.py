"""Pure TF publisher ownership contract for supported odometry modes."""

from __future__ import annotations


class TfOwnershipError(RuntimeError):
    pass


def expected_tf_owners(
    odometry_mode: str, structure_tf_source: str = "isaac"
) -> dict[str, str]:
    if odometry_mode == "ideal":
        odom_owner = "isaac"
    elif odometry_mode == "mixed":
        odom_owner = "isaac_compute_odometry"
    elif odometry_mode in {"realistic", "estimated"}:
        odom_owner = "robot_localization"
    else:
        raise TfOwnershipError(f"unknown odometry mode {odometry_mode!r}")
    if structure_tf_source not in {"isaac", "rsp"}:
        raise TfOwnershipError(
            f"unknown structure TF source {structure_tf_source!r}"
        )
    if odometry_mode in {"ideal", "mixed"} and structure_tf_source == "rsp":
        raise TfOwnershipError(
            f"{odometry_mode} odometry requires Isaac-owned structure TF"
        )
    return {
        "map->odom": (
            "ideal_localization_tf"
            if odometry_mode == "ideal"
            else "amcl"
        ),
        "odom->base_link": odom_owner,
        "base_link->wheel_links": structure_tf_source,
        "base_link->sensor_links": structure_tf_source,
        "camera_links->optical_frames": structure_tf_source,
    }


def validate_tf_publishers(
    odometry_mode: str,
    publishers: dict[str, list[str]],
    structure_tf_source: str = "isaac",
) -> None:
    expected = expected_tf_owners(odometry_mode, structure_tf_source)
    for transform, owner in expected.items():
        actual = publishers.get(transform, [])
        if actual != [owner]:
            raise TfOwnershipError(f"{transform} must have sole owner {owner!r}, got {actual}")
    if any("world" in transform.lower() for transform in publishers):
        raise TfOwnershipError("ROS world frame is forbidden")
