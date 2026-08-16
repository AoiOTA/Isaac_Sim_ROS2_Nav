"""Pure TF publisher ownership contract for ideal and realistic modes.

The ``map->odom`` owner is selected by ``localization_backend``:

- ``ideal`` -> ``ideal_localization_tf`` (the robot_bringup ideal
  localization TF publisher).  This intentionally corrects the previously
  hardcoded ``slam_toolbox`` owner, which was stale: in ideal mode the real
  ``map->odom`` publisher has always been ``ideal_localization_tf``.
- ``amcl`` -> ``localization_continuity_guard`` (AMCL itself no longer
  broadcasts TF; the guard filters ``/amcl_pose`` into the sole ``map->odom``)
- ``slam_toolbox`` -> ``slam_toolbox``

When ``localization_backend`` is ``None`` the backend is derived from the
odometry mode: ``ideal`` -> ``ideal``, ``realistic`` -> ``slam_toolbox``.
"""

from __future__ import annotations


LOCALIZATION_BACKEND_MAP_ODOM_OWNERS = {
    "ideal": "ideal_localization_tf",
    "amcl": "localization_continuity_guard",
    "slam_toolbox": "slam_toolbox",
}

_DEFAULT_LOCALIZATION_BACKEND = {
    "ideal": "ideal",
    "realistic": "slam_toolbox",
}


class TfOwnershipError(RuntimeError):
    pass


def expected_tf_owners(
    odometry_mode: str,
    structure_tf_source: str = "isaac",
    localization_backend: str | None = None,
) -> dict[str, str]:
    if odometry_mode == "ideal":
        odom_owner = "isaac"
    elif odometry_mode == "realistic":
        odom_owner = "robot_localization"
    else:
        raise TfOwnershipError(f"unknown odometry mode {odometry_mode!r}")
    if structure_tf_source not in {"isaac", "rsp"}:
        raise TfOwnershipError(
            f"unknown structure TF source {structure_tf_source!r}"
        )
    if odometry_mode == "ideal" and structure_tf_source == "rsp":
        raise TfOwnershipError(
            "ideal odometry requires Isaac-owned structure TF"
        )
    if localization_backend is None:
        localization_backend = _DEFAULT_LOCALIZATION_BACKEND[odometry_mode]
    if localization_backend not in LOCALIZATION_BACKEND_MAP_ODOM_OWNERS:
        raise TfOwnershipError(
            f"unknown localization backend {localization_backend!r}"
        )
    return {
        "map->odom": LOCALIZATION_BACKEND_MAP_ODOM_OWNERS[localization_backend],
        "odom->base_link": odom_owner,
        "base_link->wheel_links": structure_tf_source,
        "base_link->sensor_links": structure_tf_source,
        "camera_links->optical_frames": structure_tf_source,
    }


def validate_tf_publishers(
    odometry_mode: str,
    publishers: dict[str, list[str]],
    structure_tf_source: str = "isaac",
    localization_backend: str | None = None,
) -> None:
    expected = expected_tf_owners(
        odometry_mode, structure_tf_source, localization_backend
    )
    for transform, owner in expected.items():
        actual = publishers.get(transform, [])
        if actual != [owner]:
            raise TfOwnershipError(f"{transform} must have sole owner {owner!r}, got {actual}")
    if any("world" in transform.lower() for transform in publishers):
        raise TfOwnershipError("ROS world frame is forbidden")
