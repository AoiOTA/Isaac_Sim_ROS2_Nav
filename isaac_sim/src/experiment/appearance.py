"""Session-Layer-only Kujiale appearance perturbations.

The parser and colour transform are intentionally independent of Kit/PXR so
they can be validated in ordinary Python.  USD imports are delayed until the
runtime manager is constructed by :mod:`navigation_sim` after Kit starts.
"""

from __future__ import annotations

from dataclasses import dataclass
import colorsys
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Mapping

from isaac_sim.src.yaml_utils import (
    load_mapping,
    reject_unknown,
    require_keys,
    require_number,
)


PROFILE_IDS = (
    "baseline",
    "dim_warm",
    "dim_cool",
    "bright_warm",
    "bright_cool",
)
_PROFILE_ID = re.compile(r"^[a-z][a-z0-9_]*$")
_MATERIAL_COLOR_NAMES = frozenset(
    {
        "basecolor",
        "base_color",
        "diffusecolor",
        "diffuse_color",
        "albedocolor",
        "albedo_color",
        "displaycolor",
        "display_color",
    }
)


@dataclass(frozen=True)
class AppearanceProfile:
    profile_id: str
    light_intensity_scale: float
    color_temperature_k: int | None
    material_hue_shift_deg: float


@dataclass(frozen=True)
class AppearanceProfiles:
    source_path: Path
    sha256: str
    profiles: Mapping[str, AppearanceProfile]

    def require(self, profile_id: str) -> AppearanceProfile:
        try:
            return self.profiles[profile_id]
        except KeyError as exc:
            raise ValueError(
                f"unknown appearance profile {profile_id!r}; available={sorted(self.profiles)}"
            ) from exc


def _finite(value: Any, context: str, *, positive: bool = False) -> float:
    parsed = require_number(value, context=context, positive=positive)
    if not math.isfinite(parsed):
        raise ValueError(f"{context} must be finite")
    return parsed


def load_appearance_profiles(path: str | Path) -> AppearanceProfiles:
    """Load the fixed five-profile campaign contract without importing PXR."""
    source = Path(path).expanduser().resolve()
    document = load_mapping(source)
    reject_unknown(document, {"schema_version", "profiles"}, context="appearance profile document")
    require_keys(document, {"schema_version", "profiles"}, context="appearance profile document")
    if document["schema_version"] != 1:
        raise ValueError("appearance profile schema_version must be 1")
    raw_profiles = document["profiles"]
    if not isinstance(raw_profiles, dict):
        raise ValueError("appearance profiles must be a mapping")
    if tuple(sorted(raw_profiles)) != tuple(sorted(PROFILE_IDS)):
        raise ValueError(f"appearance profiles must be exactly {PROFILE_IDS}")
    profiles: dict[str, AppearanceProfile] = {}
    for identifier in PROFILE_IDS:
        if not _PROFILE_ID.fullmatch(identifier):
            raise ValueError(f"invalid appearance profile id: {identifier!r}")
        raw = raw_profiles[identifier]
        if not isinstance(raw, dict):
            raise ValueError(f"appearance profile {identifier} must be a mapping")
        reject_unknown(
            raw,
            {"light_intensity_scale", "color_temperature_k", "material_hue_shift_deg"},
            context=f"appearance profile {identifier}",
        )
        require_keys(
            raw,
            {"light_intensity_scale", "color_temperature_k", "material_hue_shift_deg"},
            context=f"appearance profile {identifier}",
        )
        scale = _finite(raw["light_intensity_scale"], f"{identifier}.light_intensity_scale", positive=True)
        if not 0.1 <= scale <= 2.0:
            raise ValueError(f"{identifier}.light_intensity_scale must be between 0.1 and 2.0")
        temperature = raw["color_temperature_k"]
        if temperature is not None:
            if isinstance(temperature, bool) or not isinstance(temperature, int) or not 1000 <= temperature <= 12000:
                raise ValueError(f"{identifier}.color_temperature_k must be null or an integer from 1000 to 12000")
        hue = _finite(raw["material_hue_shift_deg"], f"{identifier}.material_hue_shift_deg")
        if not -180.0 <= hue <= 180.0:
            raise ValueError(f"{identifier}.material_hue_shift_deg must be between -180 and 180")
        profiles[identifier] = AppearanceProfile(identifier, scale, temperature, hue)
    baseline = profiles["baseline"]
    if baseline != AppearanceProfile("baseline", 1.0, None, 0.0):
        raise ValueError("baseline appearance profile must not author an override")
    return AppearanceProfiles(source, hashlib.sha256(source.read_bytes()).hexdigest(), profiles)


def rotate_rgb_hue(rgb: tuple[float, float, float], degrees: float) -> tuple[float, float, float]:
    """Rotate an RGB colour's HSV hue, preserving saturation and value."""
    if len(rgb) != 3 or not all(math.isfinite(float(value)) for value in rgb):
        raise ValueError("rgb must contain three finite values")
    red, green, blue = (max(0.0, min(1.0, float(value))) for value in rgb)
    hue, saturation, value = colorsys.rgb_to_hsv(red, green, blue)
    shifted = (hue + degrees / 360.0) % 1.0
    return tuple(float(value) for value in colorsys.hsv_to_rgb(shifted, saturation, value))


def is_material_color_input(attribute_name: str) -> bool:
    """Return whether a shader attribute is a supported authored colour input."""
    if not attribute_name.startswith("inputs:"):
        return False
    name = attribute_name.removeprefix("inputs:").replace("-", "_").lower()
    compact = name.replace("_", "")
    return name in _MATERIAL_COLOR_NAMES or compact in {
        item.replace("_", "") for item in _MATERIAL_COLOR_NAMES
    }


class AppearanceManager:
    """Apply named appearance profiles in a dedicated anonymous session layer."""

    def __init__(self, stage: object, profiles: AppearanceProfiles) -> None:
        from pxr import Sdf

        self._stage = stage
        self._profiles = profiles
        self._layer = Sdf.Layer.CreateAnonymous("kujiale_appearance_overrides")
        self._session_layer = stage.GetSessionLayer()
        if self._layer.identifier not in self._session_layer.subLayerPaths:
            self._session_layer.subLayerPaths.append(self._layer.identifier)
        self._publisher = None
        self._node = None
        self._active_profile_id: str | None = None
        self._inventory = self._build_inventory()

    @property
    def active_profile_id(self) -> str | None:
        return self._active_profile_id

    @property
    def state(self) -> dict[str, Any]:
        profile_id = self._active_profile_id
        if profile_id is None:
            raise RuntimeError("appearance profile has not been applied")
        profile = self._profiles.require(profile_id)
        return {
            "schema_version": 1,
            "profile_id": profile.profile_id,
            "config_path": str(self._profiles.source_path),
            "config_sha256": self._profiles.sha256,
            "session_layer_identifier": self._layer.identifier,
            "inventory": dict(self._inventory),
            "overrides": {
                "light_intensity_scale": profile.light_intensity_scale,
                "color_temperature_k": profile.color_temperature_k,
                "material_hue_shift_deg": profile.material_hue_shift_deg,
            },
        }

    def _build_inventory(self) -> dict[str, Any]:
        from pxr import UsdLux

        lights: list[str] = []
        material_inputs: list[str] = []
        for prim in self._stage.Traverse():
            light = UsdLux.LightAPI(prim)
            intensity = light.GetIntensityAttr()
            if intensity and intensity.IsValid():
                lights.append(str(prim.GetPath()))
            for attribute in prim.GetAttributes():
                if is_material_color_input(attribute.GetName()) and attribute.IsValid():
                    value = attribute.Get()
                    if self._rgb_value(value) is not None:
                        material_inputs.append(f"{prim.GetPath()}.{attribute.GetName()}")
        payload = {"lights": sorted(lights), "material_color_inputs": sorted(material_inputs)}
        return {
            "light_count": len(payload["lights"]),
            "material_color_input_count": len(payload["material_color_inputs"]),
            "sha256": hashlib.sha256(
                json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
            ).hexdigest(),
        }

    @staticmethod
    def _rgb_value(value: Any) -> tuple[float, float, float] | None:
        if value is None:
            return None
        try:
            values = tuple(float(component) for component in value)
        except (TypeError, ValueError):
            return None
        if len(values) not in {3, 4} or not all(math.isfinite(component) for component in values):
            return None
        return values[:3]

    def _apply_profile(self, profile: AppearanceProfile) -> tuple[int, int]:
        from pxr import Sdf, Usd, UsdLux

        self._layer.Clear()
        if profile.profile_id == "baseline":
            return 0, 0
        if self._inventory["light_count"] == 0:
            raise RuntimeError("appearance profile cannot apply: no authored USD light intensity inputs found")
        if self._inventory["material_color_input_count"] == 0:
            raise RuntimeError("appearance profile cannot apply: no supported material colour inputs found")
        changed_lights = 0
        changed_materials = 0
        with Sdf.ChangeBlock(), Usd.EditContext(self._stage, self._layer):
            for prim in self._stage.Traverse():
                light = UsdLux.LightAPI(prim)
                intensity = light.GetIntensityAttr()
                if intensity and intensity.IsValid():
                    current = intensity.Get()
                    if isinstance(current, (int, float)) and math.isfinite(float(current)):
                        intensity.Set(float(current) * profile.light_intensity_scale)
                        if profile.color_temperature_k is not None:
                            light.CreateEnableColorTemperatureAttr(True)
                            light.CreateColorTemperatureAttr(float(profile.color_temperature_k))
                        changed_lights += 1
                for attribute in prim.GetAttributes():
                    if not is_material_color_input(attribute.GetName()):
                        continue
                    current = attribute.Get()
                    rgb = self._rgb_value(current)
                    if rgb is None:
                        continue
                    rotated = rotate_rgb_hue(rgb, profile.material_hue_shift_deg)
                    values = tuple(current)
                    try:
                        replacement = type(current)(*rotated, *values[3:])
                    except TypeError:
                        continue
                    attribute.Set(replacement)
                    changed_materials += 1
        return changed_lights, changed_materials

    def apply(self, profile_id: str) -> dict[str, Any]:
        profile = self._profiles.require(profile_id)
        changed_lights, changed_materials = self._apply_profile(profile)
        self._active_profile_id = profile.profile_id
        state = self.state
        state["applied_counts"] = {
            "lights": changed_lights,
            "material_color_inputs": changed_materials,
        }
        self._publish_state(state)
        return state

    def bind_ros(self, node: object, initial_profile_id: str) -> None:
        """Expose profile selection as an Isaac node parameter and state topic."""
        from rcl_interfaces.msg import SetParametersResult
        from rcl_interfaces.msg import ParameterDescriptor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self._node = node
        read_only = ParameterDescriptor(read_only=True)
        node.declare_parameter("appearance_config_sha256", self._profiles.sha256, read_only)
        node.declare_parameter("appearance_config_path", str(self._profiles.source_path), read_only)
        node.declare_parameter("appearance_inventory_sha256", str(self._inventory["sha256"]), read_only)
        node.declare_parameter("appearance_light_count", int(self._inventory["light_count"]), read_only)
        node.declare_parameter("appearance_material_color_input_count", int(self._inventory["material_color_input_count"]), read_only)
        node.declare_parameter("appearance_profile_id", initial_profile_id)
        self._publisher = node.create_publisher(
            String,
            "/experiment/appearance/state",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        def on_set(parameters: list[object]) -> SetParametersResult:
            requested = [item.value for item in parameters if item.name == "appearance_profile_id"]
            if not requested:
                return SetParametersResult(successful=True)
            if len(requested) != 1 or not isinstance(requested[0], str):
                return SetParametersResult(successful=False, reason="appearance_profile_id must be one string")
            try:
                self.apply(requested[0])
            except Exception as exc:
                return SetParametersResult(successful=False, reason=f"appearance profile rejected: {exc}")
            return SetParametersResult(successful=True)

        node.add_on_set_parameters_callback(on_set)
        self.apply(initial_profile_id)

    def _publish_state(self, state: Mapping[str, Any]) -> None:
        if self._publisher is None:
            return
        from std_msgs.msg import String

        message = String()
        message.data = json.dumps(state, sort_keys=True, separators=(",", ":"))
        self._publisher.publish(message)

    def close(self) -> None:
        if self._layer.identifier in self._session_layer.subLayerPaths:
            self._session_layer.subLayerPaths.remove(self._layer.identifier)
