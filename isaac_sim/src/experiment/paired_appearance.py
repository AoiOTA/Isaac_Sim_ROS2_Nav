"""Capture deterministic appearance pairs without advancing simulation time."""

from __future__ import annotations

import json
import math

import numpy as np


BASELINE_TOPIC = "/experiment/paired_appearance/baseline/image_raw"
APPEARANCE_TOPIC = "/experiment/paired_appearance/variant/image_raw"
STATE_TOPIC = "/experiment/paired_appearance/state"


def stamp_parts(simulation_time_s: float) -> tuple[int, int]:
    if not math.isfinite(simulation_time_s) or simulation_time_s < 0.0:
        raise ValueError("paired appearance simulation time must be finite and non-negative")
    seconds = int(math.floor(simulation_time_s))
    nanoseconds = int(round((simulation_time_s - seconds) * 1.0e9))
    if nanoseconds == 1_000_000_000:
        seconds += 1
        nanoseconds = 0
    return seconds, nanoseconds


def _temperature_gains(color_temperature_k: float | None) -> np.ndarray:
    """Return deterministic RGB white-balance gains relative to 6500 K."""
    if color_temperature_k is None:
        return np.ones(3, dtype=np.float32)
    temperature = float(np.clip(color_temperature_k, 3000.0, 7500.0))
    normalized = (temperature - 6500.0) / 3500.0
    return np.asarray(
        [1.0 - 0.18 * normalized, 1.0, 1.0 + 0.22 * normalized],
        dtype=np.float32,
    )


def apply_photometric_profile(image: np.ndarray, profile: object) -> np.ndarray:
    """Apply a frozen brightness, hue and white-balance profile to RGB8."""
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("paired appearance source must be HxWx3 RGB8")
    rgb = image.astype(np.float32) / 255.0
    # Rotate chroma in YIQ space while preserving luminance. This is fully
    # deterministic and avoids any dependency on the renderer's mutable USD
    # session-layer synchronization path.
    rgb_to_yiq = np.asarray(
        [[0.299, 0.587, 0.114], [0.596, -0.274, -0.322], [0.211, -0.523, 0.312]],
        dtype=np.float32,
    )
    yiq_to_rgb = np.linalg.inv(rgb_to_yiq).astype(np.float32)
    yiq = rgb @ rgb_to_yiq.T
    radians = math.radians(float(profile.material_hue_shift_deg))
    cosine = math.cos(radians)
    sine = math.sin(radians)
    chroma_i = yiq[..., 1].copy()
    chroma_q = yiq[..., 2].copy()
    yiq[..., 1] = cosine * chroma_i - sine * chroma_q
    yiq[..., 2] = sine * chroma_i + cosine * chroma_q
    transformed = yiq @ yiq_to_rgb.T
    transformed *= float(profile.light_intensity_scale)
    transformed *= _temperature_gains(profile.color_temperature_k)
    return np.rint(np.clip(transformed, 0.0, 1.0) * 255.0).astype(np.uint8)


class PairedAppearanceCapture:
    """Publish baseline/variant RGB at one identical simulation timestamp."""

    def __init__(
        self,
        *,
        node: object,
        render_product: object,
        appearance_manager: object,
        appearance_profiles: object,
        width: int,
        height: int,
        capture_rate_hz: float = 5.0,
    ) -> None:
        if width <= 0 or height <= 0 or capture_rate_hz <= 0.0:
            raise ValueError("paired appearance capture dimensions/rate must be positive")
        import omni.replicator.core as rep
        from rcl_interfaces.msg import SetParametersResult
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        from std_msgs.msg import String

        self._node = node
        self._manager = appearance_manager
        self._profiles = appearance_profiles
        self._width = int(width)
        self._height = int(height)
        self._period_s = 1.0 / float(capture_rate_hz)
        self._next_capture_s: float | None = None
        self._last_update_s: float | None = None
        self._variant_profile = ""
        self._capture_count = 0
        self._annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self._annotator.attach(render_product)
        image_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._baseline_publisher = node.create_publisher(
            Image, BASELINE_TOPIC, image_qos
        )
        self._appearance_publisher = node.create_publisher(
            Image, APPEARANCE_TOPIC, image_qos
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = node.create_publisher(String, STATE_TOPIC, state_qos)
        node.declare_parameter("paired_appearance_profile_id", "")

        def on_set(parameters: list[object]) -> SetParametersResult:
            requested = [
                item.value
                for item in parameters
                if item.name == "paired_appearance_profile_id"
            ]
            if not requested:
                return SetParametersResult(successful=True)
            if len(requested) != 1 or not isinstance(requested[0], str):
                return SetParametersResult(
                    successful=False,
                    reason="paired_appearance_profile_id must be one string",
                )
            profile_id = requested[0]
            if profile_id:
                try:
                    appearance_profiles.require(profile_id)
                except Exception as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
                if profile_id == "baseline":
                    return SetParametersResult(
                        successful=False,
                        reason="paired appearance variant cannot be baseline",
                    )
            self._variant_profile = profile_id
            self._next_capture_s = None
            self._publish_state()
            return SetParametersResult(successful=True)

        node.add_on_set_parameters_callback(on_set)
        self._publish_state()

    def _publish_state(self) -> None:
        from std_msgs.msg import String

        message = String()
        message.data = json.dumps(
            {
                "schema": "bio_nav_paired_appearance_capture_v1",
                "capture_mode": "deterministic_rgb_photometric_v1",
                "baseline_profile_id": "baseline",
                "variant_profile_id": self._variant_profile,
                "capture_rate_hz": 1.0 / self._period_s,
                "capture_count": self._capture_count,
                "appearance_config_sha256": self._profiles.sha256,
                "simulation_time_advanced_during_capture": False,
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        self._state_publisher.publish(message)

    def _annotator_rgb(self) -> np.ndarray:
        image = np.asarray(self._annotator.get_data())
        if image.shape[:2] != (self._height, self._width) or image.ndim != 3:
            raise RuntimeError(
                f"paired appearance RGB shape mismatch: {image.shape}"
            )
        if image.shape[2] < 3:
            raise RuntimeError("paired appearance RGB annotator returned fewer than 3 channels")
        # Annotator storage is reused by the next renderer step. A contiguous
        # view alone can still alias that buffer, so the captured source must
        # own an explicit snapshot before the simulation continues.
        return np.ascontiguousarray(image[..., :3], dtype=np.uint8).copy()

    def _message(self, image: np.ndarray, simulation_time_s: float):
        from sensor_msgs.msg import Image

        seconds, nanoseconds = stamp_parts(simulation_time_s)
        message = Image()
        message.header.stamp.sec = seconds
        message.header.stamp.nanosec = nanoseconds
        message.header.frame_id = "camera_front_optical_frame"
        message.height = self._height
        message.width = self._width
        message.encoding = "rgb8"
        message.is_bigendian = False
        message.step = self._width * 3
        message.data = image.tobytes()
        return message

    def update(self, simulation_time_s: float) -> bool:
        if not self._variant_profile:
            return False
        if (
            self._last_update_s is not None
            and simulation_time_s + 1.0e-9 < self._last_update_s
        ):
            self._next_capture_s = None
        self._last_update_s = simulation_time_s
        if self._next_capture_s is not None and simulation_time_s + 1.0e-9 < self._next_capture_s:
            return False
        original_profile = str(self._manager.active_profile_id or "baseline")
        if original_profile != "baseline":
            raise RuntimeError(
                "paired appearance capture requires a baseline authority route"
            )
        baseline = self._annotator_rgb()
        profile = self._profiles.require(self._variant_profile)
        appearance = apply_photometric_profile(baseline, profile)
        if np.array_equal(baseline, appearance):
            raise RuntimeError(
                "paired appearance transform produced identical baseline/variant RGB"
            )
        self._baseline_publisher.publish(self._message(baseline, simulation_time_s))
        self._appearance_publisher.publish(self._message(appearance, simulation_time_s))
        self._capture_count += 1
        self._next_capture_s = simulation_time_s + self._period_s
        if self._capture_count == 1 or self._capture_count % 50 == 0:
            self._publish_state()
        return True

    def close(self) -> None:
        if self._annotator is not None:
            self._annotator.detach()
            self._annotator = None


__all__ = [
    "APPEARANCE_TOPIC",
    "BASELINE_TOPIC",
    "PairedAppearanceCapture",
    "STATE_TOPIC",
    "apply_photometric_profile",
    "stamp_parts",
]
