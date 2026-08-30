#!/usr/bin/env python3
"""Evidence-oriented route analyzer for carrot lane-mode path offsets.

The analyzer deliberately separates what an rlog proves from what can only be
inferred.  Current validation builds publish ``staticPathOffset``,
``dynamicLaneOffset``, and ``pathBeforeStaticOffset`` in ``lateralPlan`` so the
static offset can be checked directly.  Older routes lack those fields; their
legacy reconstruction is accepted only when the route code matches the local
planner and the pre-static path can be recovered unambiguously.

With ``--baseline-log``, the positional log is treated as the +10 cm variant.
Strict straight-lane samples from the two routes are paired by GPS position,
travel heading, and speed, then reduced to independent spatial bins before a
physical vehicle-to-lane displacement is reported. A single log never proves
physical displacement.

Coordinate/sign conventions used by the current carrot implementation:

* path Y and curvature: positive is right
* Hyundai torque/steering-wheel angle: negative is right

Examples:

  ./lane_offset_route_analyzer.py /data/media/0/realdata/<route>--0/rlog.zst
  ./lane_offset_route_analyzer.py variant/rlog.zst --baseline-log baseline/rlog.zst
  ./lane_offset_route_analyzer.py 'dongle|2026-01-02--03-04-05/0' --json
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections.abc import Callable, Iterable, Sequence
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field, fields
import json
import math
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

import numpy as np


NAN = float("nan")
TRAJECTORY_SIZE = 33
CONTROL_N = 17
STATIC_TOLERANCE_M = 0.003
MPC_REPLAY_RMSE_LIMIT = 5e-5
RIGHT_CURVATURE_EPS = 1e-7
EARTH_RADIUS_M = 6_371_000.0
DIRECT_OFFSET_TOLERANCE_M = 0.003
ZERO_DYNAMIC_TOLERANCE_M = 0.005
LANE_PROBABILITY_MIN = 0.8
LANE_STD_MAX_M = 0.2
LANE_WIDTH_MIN_M = 2.6
LANE_WIDTH_MAX_M = 4.2
LANE_FIT_SPREAD_MAX_M = 0.15
PHYSICAL_STRAIGHT_CURVATURE_MAX = 0.001
PHYSICAL_MIN_SPEED_MS = 5.0
CODE_PATHS = (
  "openpilot/cereal/log.capnp",
  "openpilot/selfdrive/controls/lib/lateral_planner.py",
  "openpilot/selfdrive/controls/lib/lane_planner_2.py",
  "openpilot/selfdrive/controls/lib/lateral_mpc_lib/lat_mpc.py",
  "openpilot/selfdrive/controls/controlsd.py",
  "openpilot/selfdrive/controls/lib/latcontrol_torque.py",
)

_DYNAMIC_OFFSET_RE = re.compile(r"\boffset=([+-]?[0-9]+(?:\.[0-9]+)?)cm\b")
_TURN_SPEED_RE = re.compile(r"\bturn=([+-]?[0-9]+(?:\.[0-9]+)?)km/h\b")


@dataclass
class StageEvidence:
  status: str
  summary: str
  sample_count: int = 0
  details: dict[str, Any] = field(default_factory=dict)


@dataclass
class MpcReplayResult:
  usable: bool
  observed_rmse: float = NAN
  right_curvature_delta: float = NAN
  right_curvature_peak: float = NAN
  reason: str = ""


@dataclass
class FrameEvidence:
  mono_time: int
  model_mono_time: int
  configured_path_offset_m: float
  path_offset_m: float
  adjust_lane_offset_m: float
  dynamic_offset_m: float
  static_residual_m: float
  static_max_error_m: float
  base_target_near_m: float
  lane_center_curvature: float
  model_desired_curvature: float
  plan_early_curvature: float
  plan_peak_curvature: float
  geometry_exact: bool
  evidence_source: str
  straight: bool
  notes: list[str] = field(default_factory=list)
  # These objects are intentionally excluded from serialized output.
  _plan: Any = field(default=None, repr=False)
  _model: Any = field(default=None, repr=False)
  _car_params: Any = field(default=None, repr=False)
  _params: dict[str, str] = field(default_factory=dict, repr=False)
  _expected_pre_static: np.ndarray | None = field(default=None, repr=False)

  def public_dict(self) -> dict[str, Any]:
    result = {
      item.name: getattr(self, item.name)
      for item in fields(self)
      if not item.name.startswith("_")
    }
    return _json_finite(result)


@dataclass(frozen=True)
class LaneCenterFit:
  lane_center_y_m: float
  ego_right_of_lane_center_m: float
  lane_width_m: float
  center_curvature: float
  robust_spread_m: float


@dataclass(frozen=True)
class LocationFix:
  mono_time: int
  latitude: float
  longitude: float
  heading_deg: float
  speed_ms: float
  source: str


@dataclass(frozen=True)
class PhysicalLaneSample:
  mono_time: int
  latitude: float
  longitude: float
  heading_deg: float
  speed_ms: float
  ego_right_m: float
  lane_center_y_m: float
  lane_width_m: float
  fit_spread_m: float
  static_offset_m: float
  dynamic_offset_m: float
  location_source: str


@dataclass
class PhysicalSampleSet:
  samples: list[PhysicalLaneSample]
  rejections: Counter[str]
  location_source: str
  location_fix_count: int
  dongle_ids: tuple[str, ...]
  car_fingerprints: tuple[str, ...]


@dataclass
class LaneOffsetReport:
  source: str
  expected_path_offset_m: float
  branch: str
  commit: str
  code_compatible: bool | None
  stages: dict[str, StageEvidence]
  frames: list[FrameEvidence]
  observability_gaps: list[str]
  diagnostics: list[str]

  def to_dict(self, include_frames: bool = False) -> dict[str, Any]:
    data = {
      "source": self.source,
      "expected_path_offset_m": self.expected_path_offset_m,
      "branch": self.branch,
      "commit": self.commit,
      "code_compatible": self.code_compatible,
      "stages": {name: asdict(stage) for name, stage in self.stages.items()},
      "observability_gaps": self.observability_gaps,
      "diagnostics": self.diagnostics,
    }
    if include_frames:
      data["frames"] = [frame.public_dict() for frame in self.frames]
    return _json_finite(data)


def _json_finite(value: Any) -> Any:
  if isinstance(value, float) and not math.isfinite(value):
    return None
  if isinstance(value, dict):
    return {key: _json_finite(item) for key, item in value.items()}
  if isinstance(value, list):
    return [_json_finite(item) for item in value]
  return value


def _which(event: Any) -> str:
  value = event.which()
  return str(value)


def _payload(event: Any, name: str) -> Any:
  return getattr(event, name)


def _nested(obj: Any, path: str, default: Any = None) -> Any:
  try:
    for part in path.split("."):
      obj = getattr(obj, part)
    return obj
  except (AttributeError, TypeError, RuntimeError):
    return default


def _enum_name(value: Any) -> str:
  text = str(value)
  return text.rsplit(".", 1)[-1]


def _float_sequence(value: Any) -> np.ndarray:
  try:
    return np.asarray(list(value), dtype=float)
  except (TypeError, ValueError):
    return np.asarray([], dtype=float)


def _decode_param(value: Any) -> str:
  if value is None:
    return ""
  try:
    raw = bytes(value)
  except (TypeError, ValueError):
    raw = value
  if isinstance(raw, bytes):
    return raw.decode("utf-8", errors="replace").rstrip("\x00")
  return str(raw)


def _params_from_init(init_data: Any) -> dict[str, str]:
  result: dict[str, str] = {}
  entries = _nested(init_data, "params.entries", [])
  for entry in entries:
    key = str(_nested(entry, "key", ""))
    if key:
      result[key] = _decode_param(_nested(entry, "value", b""))
  return result


def _param_float(params: dict[str, str], key: str, scale: float = 1.0) -> float:
  try:
    return float(params[key]) * scale
  except (KeyError, TypeError, ValueError):
    return NAN


def _latest_before(records: Sequence[tuple[int, Any]], mono_time: int, max_age_ns: int | None = None) -> Any | None:
  if not records:
    return None
  times = [record[0] for record in records]
  index = bisect_right(times, mono_time) - 1
  if index < 0:
    return None
  if max_age_ns is not None and mono_time - records[index][0] > max_age_ns:
    return None
  return records[index][1]


def _latest_record_before(records: Sequence[tuple[int, Any]], mono_time: int,
                          max_age_ns: int | None = None) -> tuple[int, Any] | None:
  if not records:
    return None
  times = [record[0] for record in records]
  index = bisect_right(times, mono_time) - 1
  if index < 0:
    return None
  record = records[index]
  if max_age_ns is not None and mono_time - record[0] > max_age_ns:
    return None
  return record


def _nearest_record(records: Sequence[tuple[int, Any]], mono_time: int,
                    max_delta_ns: int) -> tuple[int, Any] | None:
  if not records:
    return None
  times = [record[0] for record in records]
  insertion = bisect_right(times, mono_time)
  candidates = []
  if insertion:
    candidates.append(records[insertion - 1])
  if insertion < len(records):
    candidates.append(records[insertion])
  if not candidates:
    return None
  result = min(candidates, key=lambda record: abs(record[0] - mono_time))
  return result if abs(result[0] - mono_time) <= max_delta_ns else None


def _heading_difference_deg(left: float, right: float) -> float:
  return abs((left - right + 180.0) % 360.0 - 180.0)


def parse_debug_offsets(text: str) -> tuple[float, float]:
  """Return the applied dynamic lane offset [m] and turn speed [km/h]."""
  offset_match = _DYNAMIC_OFFSET_RE.search(text)
  turn_match = _TURN_SPEED_RE.search(text)
  dynamic = float(offset_match.group(1)) * 0.01 if offset_match else NAN
  turn = float(turn_match.group(1)) if turn_match else NAN
  return dynamic, turn


def _lane_center_curvature(x: np.ndarray, y: np.ndarray) -> float:
  valid = np.isfinite(x) & np.isfinite(y) & (x >= 0.0) & (x <= 50.0)
  if np.count_nonzero(valid) < 6 or np.ptp(x[valid]) < 10.0:
    return NAN
  try:
    quadratic, linear, _ = np.polyfit(x[valid], y[valid], 2)
  except (ValueError, np.linalg.LinAlgError):
    return NAN
  return float((2.0 * quadratic) / ((1.0 + linear * linear) ** 1.5))


def _model_lane_center_curvature(model: Any) -> float:
  lane_lines = list(_nested(model, "laneLines", []))
  if len(lane_lines) < 3:
    return NAN
  left_x = _float_sequence(_nested(lane_lines[1], "x", []))
  left_y = _float_sequence(_nested(lane_lines[1], "y", []))
  right_y = _float_sequence(_nested(lane_lines[2], "y", []))
  if not len(left_x) or len(left_x) != len(left_y) or len(left_y) != len(right_y):
    return NAN
  return _lane_center_curvature(left_x, (left_y + right_y) * 0.5)


def _robust_quadratic_fit(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float] | None:
  valid = np.isfinite(x) & np.isfinite(y)
  x = np.asarray(x[valid], dtype=float)
  y = np.asarray(y[valid], dtype=float)
  if len(x) < 8 or np.ptp(x) < 8.0:
    return None
  x_scale = max(float(np.max(np.abs(x))), 1.0)
  normalized_x = x / x_scale
  design = np.column_stack((np.ones(len(x)), normalized_x, normalized_x**2))
  weights = np.ones(len(x), dtype=float)
  coefficients = np.zeros(3, dtype=float)
  spread = NAN
  for _ in range(8):
    weighted_design = design * np.sqrt(weights)[:, None]
    weighted_y = y * np.sqrt(weights)
    try:
      coefficients = np.linalg.lstsq(weighted_design, weighted_y, rcond=None)[0]
    except np.linalg.LinAlgError:
      return None
    residual = y - design @ coefficients
    residual_center = float(np.median(residual))
    spread = 1.4826 * float(np.median(np.abs(residual - residual_center)))
    if spread < 1e-6:
      break
    scaled_residual = np.abs(residual - residual_center) / (1.5 * spread)
    weights = np.ones(len(scaled_residual), dtype=float)
    outliers = scaled_residual > 1.0
    weights[outliers] = 1.0 / scaled_residual[outliers]
  # Convert [1, x/scale, (x/scale)^2] back to ordinary x coefficients.
  ordinary = np.array((coefficients[0], coefficients[1] / x_scale, coefficients[2] / x_scale**2))
  return ordinary, spread


def fit_ego_right_from_lane_lines(model: Any, *, max_fit_distance_m: float = 45.0) -> tuple[LaneCenterFit | None, str]:
  """Fit the two inner lane lines and evaluate their center at the camera origin.

  Carrot's model Y coordinate is positive to the vehicle's right. Therefore a
  vehicle displaced right of the lane center observes that center to its left:
  ``ego_right_of_lane_center = -lane_center_y``. A fixed camera/extrinsic
  lateral bias is present in both A/B runs and cancels in their difference.
  """
  lane_lines = list(_nested(model, "laneLines", []))
  if len(lane_lines) < 3:
    return None, "lane_lines_missing"
  left_x = _float_sequence(_nested(lane_lines[1], "x", []))
  left_y = _float_sequence(_nested(lane_lines[1], "y", []))
  right_x = _float_sequence(_nested(lane_lines[2], "x", []))
  right_y = _float_sequence(_nested(lane_lines[2], "y", []))
  if min(len(left_x), len(left_y), len(right_x), len(right_y)) < 8:
    return None, "lane_trajectory_incomplete"

  def sorted_unique(x_values: np.ndarray, y_values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    valid = np.isfinite(x_values) & np.isfinite(y_values)
    order = np.argsort(x_values[valid])
    sorted_x = x_values[valid][order]
    sorted_y = y_values[valid][order]
    unique_x, indices = np.unique(sorted_x, return_index=True)
    return unique_x, sorted_y[indices]

  left_x, left_y = sorted_unique(left_x, left_y)
  right_x, right_y = sorted_unique(right_x, right_y)
  if len(left_x) < 8 or len(right_x) < 8:
    return None, "lane_trajectory_incomplete"
  start_x = max(0.0, float(left_x[0]), float(right_x[0]))
  end_x = min(max_fit_distance_m, float(left_x[-1]), float(right_x[-1]))
  if start_x > 1.5 or end_x - start_x < 15.0:
    return None, "lane_origin_not_observed"
  grid = np.linspace(start_x, end_x, 25)
  left_grid = np.interp(grid, left_x, left_y)
  right_grid = np.interp(grid, right_x, right_y)
  center_grid = (left_grid + right_grid) * 0.5
  width_grid = right_grid - left_grid

  center_fit = _robust_quadratic_fit(grid, center_grid)
  width_fit = _robust_quadratic_fit(grid, width_grid)
  if center_fit is None or width_fit is None:
    return None, "lane_fit_failed"
  center_coefficients, center_spread = center_fit
  width_coefficients, width_spread = width_fit
  center_y = float(center_coefficients[0])
  width = float(width_coefficients[0])
  curvature = float(2.0 * center_coefficients[2] / ((1.0 + center_coefficients[1] ** 2) ** 1.5))
  robust_spread = max(float(center_spread), float(width_spread) * 0.5)
  if not LANE_WIDTH_MIN_M <= width <= LANE_WIDTH_MAX_M:
    return None, "lane_width_invalid"
  if robust_spread > LANE_FIT_SPREAD_MAX_M:
    return None, "lane_fit_noisy"
  return LaneCenterFit(center_y, -center_y, width, curvature, robust_spread), ""


def reconstruct_pre_static_lane_path(model: Any, plan: Any, dynamic_offset_m: float,
                                     lat_mpc_input_offset: float) -> tuple[np.ndarray | None, float, list[str]]:
  """Reconstruct the exact pre-PathOffset target for strict two-line frames.

  This intentionally covers only the branch where both modified inner-line
  probabilities exceed 0.7.  In that branch the filtered lane width cancels
  algebraically and the selected target is the inner-line midpoint.  Other
  branches are rejected rather than approximated.
  """
  notes: list[str] = []
  if not math.isfinite(dynamic_offset_m):
    return None, NAN, ["latDebugText does not expose the applied dynamic offset"]

  lane_lines = list(_nested(model, "laneLines", []))
  probs = _float_sequence(_nested(model, "laneLineProbs", []))
  stds = _float_sequence(_nested(model, "laneLineStds", []))
  if len(lane_lines) < 4 or len(probs) < 3 or len(stds) < 3:
    return None, NAN, ["modelV2 does not contain four lane lines/probability/std fields"]

  left = lane_lines[1]
  right = lane_lines[2]
  left_x = _float_sequence(_nested(left, "x", []))
  left_y = _float_sequence(_nested(left, "y", []))
  right_y = _float_sequence(_nested(right, "y", []))
  left_t = _float_sequence(_nested(left, "t", []))
  right_t = _float_sequence(_nested(right, "t", []))
  path_t = _float_sequence(_nested(model, "position.t", []))
  if not all(len(values) == TRAJECTORY_SIZE for values in (left_x, left_y, right_y, left_t, right_t, path_t)):
    return None, NAN, ["lane/model trajectories are not 33 points"]

  width_points = right_y - left_y
  probability_modifiers = [
    float(np.interp(t_check * (float(_nested(model, "velocity.x", [0.0])[0]) + 7.0), left_x, width_points,
                    left=width_points[0], right=width_points[-1]))
    for t_check in (0.0, 1.5, 3.0)
  ]
  width_modifier = min(float(np.interp(width, [4.5, 6.0], [1.0, 0.0])) for width in probability_modifiers)
  left_probability = float(probs[1] * width_modifier * np.interp(stds[1], [0.15, 0.3], [1.0, 0.0]))
  right_probability = float(probs[2] * width_modifier * np.interp(stds[2], [0.15, 0.3], [1.0, 0.0]))
  if left_probability <= 0.7 or right_probability <= 0.7:
    return None, NAN, [
      f"modified inner-line probabilities do not select the exact two-line branch ({left_probability:.3f}, {right_probability:.3f})"
    ]

  if _enum_name(_nested(model, "meta.desire", "none")) != "none":
    return None, NAN, ["model desire is not none"]
  if not bool(_nested(plan, "useLaneLines", False)):
    return None, NAN, ["lateralPlan.useLaneLines is false"]

  lane_t = (left_t + right_t) * 0.5
  lane_center = (left_y + right_y) * 0.5
  safe = np.isfinite(lane_t) & np.isfinite(lane_center)
  if not safe[0] or np.count_nonzero(safe) < 3:
    return None, NAN, ["inner-line midpoint is not finite"]

  query_t = path_t * (1.0 + lat_mpc_input_offset)
  center_interp = np.interp(query_t, lane_t[safe], lane_center[safe])
  expected_pre_static = center_interp + dynamic_offset_m
  lane_curvature = _lane_center_curvature(left_x, lane_center)
  return expected_pre_static, lane_curvature, notes


def _early_curvature(curvatures: np.ndarray) -> tuple[float, float]:
  window = curvatures[1:min(8, len(curvatures))]
  if not len(window):
    return NAN, NAN
  return float(np.mean(window)), float(window[np.argmax(np.abs(window))])


def _smooth_moving_avg(values: np.ndarray, window: int = 5) -> np.ndarray:
  if window < 2:
    return values
  if window % 2 == 0:
    window += 1
  pad = window // 2
  padded = np.pad(values, (pad, pad), mode="edge")
  kernel = np.ones(window) / window
  return np.convolve(padded, kernel, mode="same")[pad:-pad]


def _yaw_from_path(path_xyz: np.ndarray, v_plan: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  # Kept numerically identical to lateral_planner.yaw_from_path_no_scipy for
  # route counterfactuals without importing the planner (which loads acados).
  x = path_xyz[:, 0].astype(float)
  y = path_xyz[:, 1].astype(float)
  if len(x) < 5:
    return np.zeros(len(x)), np.zeros(len(x))
  dx = np.diff(x)
  dy = np.diff(y)
  ds_segment = np.sqrt(dx * dx + dy * dy)
  ds_segment[ds_segment < 0.05] = 0.05
  distance = np.zeros(len(x), dtype=float)
  distance[1:] = np.cumsum(ds_segment)
  if distance[-1] < 0.5:
    return np.zeros(len(x)), np.zeros(len(x))
  window = 9 if float(v_plan[0]) <= 6.0 else 5
  x_smooth = _smooth_moving_avg(x, window)
  y_smooth = _smooth_moving_avg(y, window)
  dx_ds = np.gradient(x_smooth, distance)
  dy_ds = np.gradient(y_smooth, distance)
  d2x_ds2 = np.gradient(dx_ds, distance)
  d2y_ds2 = np.gradient(dy_ds, distance)
  yaw = np.unwrap(np.arctan2(dy_ds, dx_ds))
  denominator = (dx_ds * dx_ds + dy_ds * dy_ds) ** 1.5
  denominator[denominator < 1e-9] = 1e-9
  curvature = (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / denominator
  yaw_rate = curvature * v_plan
  if float(v_plan[0]) <= 6.0:
    yaw_rate = _smooth_moving_avg(yaw_rate, 7)
  return np.where(np.isfinite(yaw), yaw, 0.0), np.clip(np.where(np.isfinite(yaw_rate), yaw_rate, 0.0), -2.0, 2.0)


def _load_lateral_mpc() -> tuple[Any, int]:
  if sys.platform == "darwin":
    import ctypes
    root = Path(__file__).resolve().parents[3]
    library_dir = root / "third_party" / "acados" / "Darwin" / "lib"
    for name in ("libqpOASES_e.3.1.dylib", "libblasfeo.dylib", "libhpipm.dylib", "libacados.dylib"):
      ctypes.CDLL(str(library_dir / name), mode=ctypes.RTLD_GLOBAL)
  from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc, N
  return LateralMpc, N


def replay_mpc_static_counterfactual(frame: FrameEvidence) -> MpcReplayResult:
  """Replay recorded target and a target with only PathOffset removed."""
  try:
    LateralMpc, horizon = _load_lateral_mpc()
    plan = frame._plan
    model = frame._model
    car_params = frame._car_params
    params = frame._params

    y_target = _float_sequence(_nested(plan, "dPathPoints", []))
    observed_curvatures = _float_sequence(_nested(plan, "curvatures", []))
    x_path = _float_sequence(_nested(model, "position.x", []))
    z_path = _float_sequence(_nested(model, "position.z", []))
    vx = _float_sequence(_nested(model, "velocity.x", []))
    vy = _float_sequence(_nested(model, "velocity.y", []))
    vz = _float_sequence(_nested(model, "velocity.z", []))
    if not all(len(values) == horizon + 1 for values in (y_target, x_path, z_path, vx, vy, vz)):
      return MpcReplayResult(False, reason="route lacks a complete 33-point MPC target/model velocity")
    if len(observed_curvatures) < CONTROL_N:
      return MpcReplayResult(False, reason="lateralPlan.curvatures is incomplete")

    weights = (
      _param_float(params, "LatMpcPathCost", 0.01),
      _param_float(params, "LatMpcMotionCost", 0.01),
      _param_float(params, "LatMpcAccelCost", 0.01),
      _param_float(params, "LatMpcJerkCost", 0.01),
      _param_float(params, "LatMpcSteeringRateCost"),
    )
    if not all(math.isfinite(value) for value in weights):
      return MpcReplayResult(False, reason="initData lacks one or more LatMpc* cost Params")

    v_plan = np.clip(np.linalg.norm(np.column_stack((vx, vy, vz)), axis=1), 1.0, np.inf)
    wheelbase = float(_nested(car_params, "wheelbase", NAN))
    center_to_front = float(_nested(car_params, "centerToFront", NAN))
    mass = float(_nested(car_params, "mass", NAN))
    rear_stiffness = float(_nested(car_params, "tireStiffnessRear", NAN))
    if not all(math.isfinite(value) and value > 0.0 for value in (wheelbase, center_to_front, mass, rear_stiffness)):
      return MpcReplayResult(False, reason="carParams lacks MPC vehicle factors")
    factor1 = wheelbase - center_to_front
    factor2 = center_to_front * mass / (wheelbase * rear_stiffness)
    lateral_factor = np.clip(factor1 - factor2 * v_plan**2, 0.0, np.inf)
    model_params = np.column_stack((v_plan, lateral_factor))

    position_x = _float_sequence(_nested(plan, "position.x", []))
    position_y = _float_sequence(_nested(plan, "position.y", []))
    psis = _float_sequence(_nested(plan, "psis", []))
    if not len(position_x) or not len(position_y) or not len(psis):
      return MpcReplayResult(False, reason="lateralPlan does not expose enough solved state to reconstruct MPC x0")
    v_div = np.maximum(v_plan[:CONTROL_N], 6.0)
    x0 = np.array((position_x[0], position_y[0], psis[0], observed_curvatures[0] * v_div[0]), dtype=float)
    target_xyz = np.column_stack((x_path, y_target, z_path))
    heading, yaw_rate = _yaw_from_path(target_xyz, v_plan)

    def solve(target_y: np.ndarray) -> np.ndarray:
      mpc = LateralMpc(x0=x0)
      mpc.set_weights(*weights)
      # SQP-RTI is warm-started in production. Repeating against the same
      # target converges both counterfactuals before comparing them.
      for _ in range(5):
        mpc.run(x0, model_params, target_y, heading, yaw_rate)
      if mpc.solution_status != 0:
        raise RuntimeError(f"acados status {mpc.solution_status}")
      return mpc.x_sol[:CONTROL_N, 3] / v_div

    with_static = solve(y_target)
    without_static = solve(y_target - frame.path_offset_m)
    observed_rmse = float(np.sqrt(np.mean((with_static - observed_curvatures[:CONTROL_N]) ** 2)))
    delta = with_static - without_static
    early_delta, peak_delta = _early_curvature(delta)
    usable = observed_rmse <= MPC_REPLAY_RMSE_LIMIT and early_delta > RIGHT_CURVATURE_EPS
    reason = "" if usable else (
      f"replay RMSE {observed_rmse:.3g} exceeds {MPC_REPLAY_RMSE_LIMIT:.3g}"
      if observed_rmse > MPC_REPLAY_RMSE_LIMIT else
      f"counterfactual early curvature delta {early_delta:.3g} is not right-positive"
    )
    return MpcReplayResult(usable, observed_rmse, early_delta, peak_delta, reason)
  except Exception as exc:  # Runtime/library gaps must become evidence gaps, not a false pass.
    return MpcReplayResult(False, reason=f"MPC replay unavailable: {exc}")


def _code_compatible_with_checkout(commit: str, repo_root: Path) -> bool | None:
  if not commit:
    return None
  try:
    subprocess.run(["git", "cat-file", "-e", f"{commit}^{{commit}}"], cwd=repo_root,
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # Compare against the actual working tree too. An uncommitted planner/schema
    # change must not be silently treated as route-compatible.
    result = subprocess.run(["git", "diff", "--quiet", commit, "--", *CODE_PATHS], cwd=repo_root)
    return result.returncode == 0
  except (OSError, subprocess.SubprocessError):
    return None


def _make_frame(plan_event: Any, records: dict[str, list[tuple[int, Any]]], models: dict[int, Any],
                params_epochs: list[tuple[int, dict[str, str]]], car_params_records: list[tuple[int, Any]],
                straight_curvature_limit: float) -> FrameEvidence:
  mono_time = int(_nested(plan_event, "logMonoTime", 0))
  plan = _payload(plan_event, "lateralPlan")
  model_mono_time = int(_nested(plan, "modelMonoTime", 0))
  model = models.get(model_mono_time)
  params = _latest_before(params_epochs, mono_time) or {}
  car_params = _latest_before(car_params_records, mono_time)
  configured_path_offset = _param_float(params, "PathOffset", 0.01)
  adjust_offset = _param_float(params, "AdjustLaneOffset", 0.01)
  lat_mpc_input_offset = _param_float(params, "LatMpcInputOffset", 0.01)
  if not math.isfinite(lat_mpc_input_offset):
    lat_mpc_input_offset = 0.0
  debug_dynamic, turn_speed = parse_debug_offsets(str(_nested(plan, "latDebugText", "")))

  notes: list[str] = []
  expected_pre_static: np.ndarray | None = None
  lane_curvature = _model_lane_center_curvature(model) if model is not None else NAN
  path_offset = configured_path_offset
  dynamic = debug_dynamic
  evidence_source = "none"

  # New routes publish all three values directly. A non-empty before-path is
  # the presence marker because old capnp messages read new scalar fields as 0.
  direct_before = _float_sequence(_nested(plan, "pathBeforeStaticOffset", []))
  observed_path = _float_sequence(_nested(plan, "dPathPoints", []))
  if len(direct_before) and len(direct_before) == len(observed_path):
    path_offset = float(_nested(plan, "staticPathOffset", NAN))
    dynamic = float(_nested(plan, "dynamicLaneOffset", NAN))
    expected_pre_static = direct_before
    evidence_source = "direct_numeric"
  if model is None:
    notes.append("no modelV2 event exactly matches lateralPlan.modelMonoTime")
  elif expected_pre_static is None:
    expected_pre_static, fallback_curvature, geometry_notes = reconstruct_pre_static_lane_path(
      model, plan, debug_dynamic, lat_mpc_input_offset,
    )
    if math.isfinite(fallback_curvature):
      lane_curvature = fallback_curvature
    notes.extend(geometry_notes)
    if expected_pre_static is not None:
      evidence_source = "legacy_reconstruction"

  static_residual = NAN
  static_max_error = NAN
  base_target_near = NAN
  if expected_pre_static is not None and len(observed_path) == len(expected_pre_static):
    residuals = observed_path - expected_pre_static
    static_residual = float(np.median(residuals))
    static_max_error = float(np.max(np.abs(residuals - path_offset))) if math.isfinite(path_offset) else NAN
    base_target_near = float(np.mean(expected_pre_static[:min(8, len(expected_pre_static))]))
  elif expected_pre_static is not None:
    notes.append("lateralPlan.dPathPoints length does not match the reconstructed target")

  model_curvature = float(_nested(model, "action.desiredCurvature", NAN)) if model is not None else NAN
  plan_curvatures = _float_sequence(_nested(plan, "curvatures", []))
  early_curvature, peak_curvature = _early_curvature(plan_curvatures)
  straight = (
    bool(_nested(plan, "useLaneLines", False))
    and math.isfinite(model_curvature) and abs(model_curvature) <= straight_curvature_limit
    and math.isfinite(lane_curvature) and abs(lane_curvature) <= straight_curvature_limit
    and (not math.isfinite(turn_speed) or abs(turn_speed) <= 5.0)
  )
  return FrameEvidence(
    mono_time=mono_time,
    model_mono_time=model_mono_time,
    configured_path_offset_m=configured_path_offset,
    path_offset_m=path_offset,
    adjust_lane_offset_m=adjust_offset,
    dynamic_offset_m=dynamic,
    static_residual_m=static_residual,
    static_max_error_m=static_max_error,
    base_target_near_m=base_target_near,
    lane_center_curvature=lane_curvature,
    model_desired_curvature=model_curvature,
    plan_early_curvature=early_curvature,
    plan_peak_curvature=peak_curvature,
    geometry_exact=expected_pre_static is not None,
    evidence_source=evidence_source,
    straight=straight,
    notes=notes,
    _plan=plan,
    _model=model,
    _car_params=car_params,
    _params=params,
    _expected_pre_static=expected_pre_static,
  )


def _ratio(values: Sequence[bool]) -> float:
  return float(sum(values) / len(values)) if values else NAN


def _median(values: Sequence[float]) -> float:
  finite = [value for value in values if math.isfinite(value)]
  return float(np.median(finite)) if finite else NAN


def _lag_adjusted_plan_curvature(frame: FrameEvidence, car_state: Any, live_delay: Any | None) -> float:
  from openpilot.selfdrive.controls.lib.drive_helpers import get_lag_adjusted_curvature

  speed = float(_nested(car_state, "vEgo", NAN))
  if not math.isfinite(speed):
    return NAN
  delay = _param_float(frame._params, "SteerActuatorDelay", 0.01)
  if not math.isfinite(delay) or delay == 0.0:
    delay = float(_nested(live_delay, "lateralDelay", NAN))
  smooth = _param_float(frame._params, "LatSmoothSec", 0.01)
  if not math.isfinite(delay) or not math.isfinite(smooth):
    return NAN
  plan = frame._plan
  try:
    return float(get_lag_adjusted_curvature(
      frame._car_params,
      speed,
      list(_nested(plan, "psis", [])),
      list(_nested(plan, "curvatures", [])),
      delay + smooth,
      list(_nested(plan, "distances", [])),
    ))
  except (TypeError, ValueError):
    return NAN


def _location_fixes(records: dict[str, list[tuple[int, Any]]]) -> tuple[list[LocationFix], str]:
  llk_fixes: list[LocationFix] = []
  for mono_time, location in records.get("liveLocationKalmanDEPRECATED", []):
    position = _float_sequence(_nested(location, "positionGeodetic.value", []))
    velocity = _float_sequence(_nested(location, "velocityNED.value", []))
    valid = (
      _enum_name(_nested(location, "status", "")) == "valid"
      and bool(_nested(location, "gpsOK", False))
      and bool(_nested(location, "inputsOK", False))
      and bool(_nested(location, "positionGeodetic.valid", False))
      and bool(_nested(location, "velocityNED.valid", False))
      and len(position) >= 2 and len(velocity) >= 2
    )
    if not valid:
      continue
    latitude, longitude = float(position[0]), float(position[1])
    speed = float(math.hypot(velocity[0], velocity[1]))
    heading = float(math.degrees(math.atan2(velocity[1], velocity[0])) % 360.0)
    if -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0 and speed >= PHYSICAL_MIN_SPEED_MS:
      llk_fixes.append(LocationFix(mono_time, latitude, longitude, heading, speed, "liveLocationKalman"))
  if llk_fixes:
    return llk_fixes, "liveLocationKalman"

  gps_fixes: list[LocationFix] = []
  for mono_time, location in records.get("gpsLocationExternal", []):
    latitude = float(_nested(location, "latitude", NAN))
    longitude = float(_nested(location, "longitude", NAN))
    speed = float(_nested(location, "speed", NAN))
    heading = float(_nested(location, "bearingDeg", NAN)) % 360.0
    horizontal_accuracy = float(_nested(location, "horizontalAccuracy", NAN))
    bearing_accuracy = float(_nested(location, "bearingAccuracyDeg", NAN))
    speed_accuracy = float(_nested(location, "speedAccuracy", NAN))
    accuracy_ok = (
      (not math.isfinite(horizontal_accuracy) or horizontal_accuracy <= 10.0)
      and (not math.isfinite(bearing_accuracy) or bearing_accuracy <= 15.0)
      and (not math.isfinite(speed_accuracy) or speed_accuracy <= 3.0)
    )
    if (
      bool(_nested(location, "hasFix", False)) and accuracy_ok
      and all(math.isfinite(value) for value in (latitude, longitude, speed, heading))
      and -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0
      and speed >= PHYSICAL_MIN_SPEED_MS
    ):
      gps_fixes.append(LocationFix(mono_time, latitude, longitude, heading, speed, "gpsLocationExternal"))
  return gps_fixes, "gpsLocationExternal" if gps_fixes else "none"


def extract_physical_lane_samples(events: Iterable[Any], *, expected_static_offset_m: float,
                                  dynamic_zero_tolerance_m: float = ZERO_DYNAMIC_TOLERANCE_M,
                                  location_max_delta_s: float = 0.20) -> PhysicalSampleSet:
  """Extract strict, directly observable straight-lane samples from one route."""
  records: dict[str, list[tuple[int, Any]]] = defaultdict(list)
  models: dict[int, Any] = {}
  plan_events: list[Any] = []
  for event in sorted(events, key=lambda item: int(_nested(item, "logMonoTime", 0))):
    if not bool(_nested(event, "valid", True)):
      continue
    try:
      name = _which(event)
    except Exception:
      continue
    mono_time = int(_nested(event, "logMonoTime", 0))
    payload = _payload(event, name)
    records[name].append((mono_time, payload))
    if name == "modelV2":
      models[mono_time] = payload
    elif name == "lateralPlan":
      plan_events.append(event)

  location_fixes, location_source = _location_fixes(records)
  location_records = [(fix.mono_time, fix) for fix in location_fixes]
  controls_by_model: dict[int, list[tuple[int, Any]]] = defaultdict(list)
  for mono_time, controls_state in records.get("controlsState", []):
    controls_by_model[int(_nested(controls_state, "lateralPlanMonoTime", 0))].append((mono_time, controls_state))

  samples: list[PhysicalLaneSample] = []
  rejections: Counter[str] = Counter()

  def reject(reason: str) -> None:
    rejections[reason] += 1

  for event in plan_events:
    mono_time = int(_nested(event, "logMonoTime", 0))
    plan = _payload(event, "lateralPlan")
    model_time = int(_nested(plan, "modelMonoTime", 0))
    model = models.get(model_time)
    if model is None:
      reject("model_link_missing")
      continue

    before_path = _float_sequence(_nested(plan, "pathBeforeStaticOffset", []))
    final_path = _float_sequence(_nested(plan, "dPathPoints", []))
    static_offset = float(_nested(plan, "staticPathOffset", NAN))
    dynamic_offset = float(_nested(plan, "dynamicLaneOffset", NAN))
    direct_ok = (
      len(before_path) == TRAJECTORY_SIZE and len(final_path) == TRAJECTORY_SIZE
      and math.isfinite(static_offset) and math.isfinite(dynamic_offset)
      and abs(static_offset - expected_static_offset_m) <= DIRECT_OFFSET_TOLERANCE_M
      and float(np.max(np.abs((final_path - before_path) - static_offset))) <= DIRECT_OFFSET_TOLERANCE_M
    )
    if not direct_ok:
      reject("direct_numeric_evidence_invalid")
      continue
    if abs(dynamic_offset) > dynamic_zero_tolerance_m:
      reject("dynamic_offset_nonzero")
      continue
    if not bool(_nested(plan, "useLaneLines", False)):
      reject("lane_mode_inactive")
      continue
    if _enum_name(_nested(plan, "laneChangeState", "")) != "off" or \
       _enum_name(_nested(model, "meta.desire", "")) != "none":
      reject("lane_change_or_desire_active")
      continue

    probabilities = _float_sequence(_nested(model, "laneLineProbs", []))
    standard_deviations = _float_sequence(_nested(model, "laneLineStds", []))
    if len(probabilities) < 3 or len(standard_deviations) < 3 or \
       min(probabilities[1], probabilities[2]) < LANE_PROBABILITY_MIN or \
       max(standard_deviations[1], standard_deviations[2]) > LANE_STD_MAX_M:
      reject("inner_lane_confidence_invalid")
      continue

    lane_fit, fit_error = fit_ego_right_from_lane_lines(model)
    if lane_fit is None:
      reject(fit_error)
      continue
    model_curvature = float(_nested(model, "action.desiredCurvature", NAN))
    if not math.isfinite(model_curvature) or abs(model_curvature) > PHYSICAL_STRAIGHT_CURVATURE_MAX or \
       abs(lane_fit.center_curvature) > PHYSICAL_STRAIGHT_CURVATURE_MAX:
      reject("road_not_straight")
      continue

    controls_record = _nearest_record(controls_by_model.get(model_time, []), mono_time, 100_000_000)
    if controls_record is None or not bool(_nested(controls_record[1], "activeLaneLine", False)):
      reject("controls_lane_mode_inactive")
      continue
    controls_time = controls_record[0]
    car_control_record = _nearest_record(records.get("carControl", []), controls_time, 50_000_000)
    car_state_record = _nearest_record(records.get("carState", []), controls_time, 100_000_000)
    if car_control_record is None or not bool(_nested(car_control_record[1], "latActive", False)):
      reject("lateral_control_inactive")
      continue
    if car_state_record is None or bool(_nested(car_state_record[1], "steeringPressed", True)):
      reject("driver_steering_override")
      continue

    location_record = _nearest_record(location_records, mono_time, int(location_max_delta_s * 1e9))
    if location_record is None:
      reject("location_match_missing")
      continue
    location = location_record[1]
    car_speed = float(_nested(car_state_record[1], "vEgo", NAN))
    if not math.isfinite(car_speed) or abs(car_speed - location.speed_ms) > 3.0:
      reject("location_vehicle_speed_disagrees")
      continue
    samples.append(PhysicalLaneSample(
      mono_time=mono_time,
      latitude=location.latitude,
      longitude=location.longitude,
      heading_deg=location.heading_deg,
      speed_ms=location.speed_ms,
      ego_right_m=lane_fit.ego_right_of_lane_center_m,
      lane_center_y_m=lane_fit.lane_center_y_m,
      lane_width_m=lane_fit.lane_width_m,
      fit_spread_m=lane_fit.robust_spread_m,
      static_offset_m=static_offset,
      dynamic_offset_m=dynamic_offset,
      location_source=location.source,
    ))
  dongle_ids = tuple(sorted({
    str(_nested(init_data, "dongleId", ""))
    for _, init_data in records.get("initData", [])
    if str(_nested(init_data, "dongleId", ""))
  }))
  car_fingerprints = tuple(sorted({
    str(_nested(car_params, "carFingerprint", ""))
    for _, car_params in records.get("carParams", [])
    if str(_nested(car_params, "carFingerprint", ""))
  }))
  return PhysicalSampleSet(
    samples, rejections, location_source, len(location_fixes), dongle_ids, car_fingerprints,
  )


def _local_metric_xy(latitude: float, longitude: float, reference_latitude: float,
                     reference_longitude: float) -> tuple[float, float]:
  north = math.radians(latitude - reference_latitude) * EARTH_RADIUS_M
  east = math.radians(longitude - reference_longitude) * EARTH_RADIUS_M * math.cos(math.radians(reference_latitude))
  return east, north


def _bootstrap_median_interval(values: np.ndarray, *, iterations: int = 4000) -> tuple[float, float]:
  if not len(values):
    return NAN, NAN
  if len(values) == 1:
    return float(values[0]), float(values[0])
  generator = np.random.default_rng(0xCA44010)
  medians = np.empty(iterations, dtype=float)
  for index in range(iterations):
    medians[index] = float(np.median(generator.choice(values, size=len(values), replace=True)))
  return float(np.percentile(medians, 2.5)), float(np.percentile(medians, 97.5))


def analyze_physical_ab_events(baseline_events: Iterable[Any], variant_events: Iterable[Any], *,
                               expected_delta_m: float = 0.10, minimum_spatial_bins: int = 8,
                               match_distance_m: float = 8.0, match_heading_deg: float = 7.0,
                               match_speed_ms: float = 2.0, spatial_bin_m: float = 20.0,
                               physical_tolerance_m: float = 0.04,
                               dynamic_zero_tolerance_m: float = ZERO_DYNAMIC_TOLERANCE_M) -> StageEvidence:
  """Compare PathOffset=0 and +10 cm routes at matched physical locations."""
  if minimum_spatial_bins < 1 or dynamic_zero_tolerance_m < 0.0 or \
     min(match_distance_m, match_heading_deg, match_speed_ms, spatial_bin_m, physical_tolerance_m) <= 0.0:
    raise ValueError("A/B sample counts and tolerances must be positive")
  baseline = extract_physical_lane_samples(
    baseline_events,
    expected_static_offset_m=0.0,
    dynamic_zero_tolerance_m=dynamic_zero_tolerance_m,
  )
  variant = extract_physical_lane_samples(
    variant_events,
    expected_static_offset_m=expected_delta_m,
    dynamic_zero_tolerance_m=dynamic_zero_tolerance_m,
  )
  details: dict[str, Any] = {
    "baseline_accepted_samples": len(baseline.samples),
    "variant_accepted_samples": len(variant.samples),
    "baseline_location_source": baseline.location_source,
    "variant_location_source": variant.location_source,
    "baseline_location_fixes": baseline.location_fix_count,
    "variant_location_fixes": variant.location_fix_count,
    "baseline_dongle_ids": list(baseline.dongle_ids),
    "variant_dongle_ids": list(variant.dongle_ids),
    "baseline_car_fingerprints": list(baseline.car_fingerprints),
    "variant_car_fingerprints": list(variant.car_fingerprints),
    "baseline_rejections": dict(baseline.rejections),
    "variant_rejections": dict(variant.rejections),
    "expected_rightward_delta_m": expected_delta_m,
    "physical_tolerance_m": physical_tolerance_m,
    "match_distance_m": match_distance_m,
    "match_heading_deg": match_heading_deg,
    "match_speed_ms": match_speed_ms,
    "spatial_bin_m": spatial_bin_m,
    "minimum_spatial_bins": minimum_spatial_bins,
    "coordinate_sign": "ego_right = -lane_center_y; a rightward vehicle shift makes the observed lane center more negative",
  }
  same_vehicle = (
    len(baseline.dongle_ids) == len(variant.dongle_ids) == 1
    and baseline.dongle_ids == variant.dongle_ids
    and len(baseline.car_fingerprints) == len(variant.car_fingerprints) == 1
    and baseline.car_fingerprints == variant.car_fingerprints
  )
  details["same_vehicle_identity"] = same_vehicle
  if not same_vehicle:
    return StageEvidence(
      "missing", "A/B routes do not establish the same dongle and vehicle fingerprint",
      0, details,
    )
  if not baseline.samples or not variant.samples:
    return StageEvidence(
      "missing", "baseline or +10 cm route lacks strict direct/GPS/lane/control samples",
      0, details,
    )

  all_samples = baseline.samples + variant.samples
  reference_latitude = float(np.median([sample.latitude for sample in all_samples]))
  reference_longitude = float(np.median([sample.longitude for sample in all_samples]))
  baseline_xy = [
    _local_metric_xy(sample.latitude, sample.longitude, reference_latitude, reference_longitude)
    for sample in baseline.samples
  ]
  variant_xy = [
    _local_metric_xy(sample.latitude, sample.longitude, reference_latitude, reference_longitude)
    for sample in variant.samples
  ]
  cell_size = max(match_distance_m, 0.1)
  baseline_cells: dict[tuple[int, int], list[int]] = defaultdict(list)
  for index, (east, north) in enumerate(baseline_xy):
    baseline_cells[(math.floor(east / cell_size), math.floor(north / cell_size))].append(index)

  matches: list[tuple[int, int, float, float, float]] = []
  for variant_index, ((east, north), variant_sample) in enumerate(zip(variant_xy, variant.samples, strict=True)):
    cell_east = math.floor(east / cell_size)
    cell_north = math.floor(north / cell_size)
    candidates: list[tuple[float, int, float, float]] = []
    for east_delta in (-1, 0, 1):
      for north_delta in (-1, 0, 1):
        for baseline_index in baseline_cells.get((cell_east + east_delta, cell_north + north_delta), []):
          baseline_east, baseline_north = baseline_xy[baseline_index]
          distance = math.hypot(east - baseline_east, north - baseline_north)
          baseline_sample = baseline.samples[baseline_index]
          heading_delta = _heading_difference_deg(variant_sample.heading_deg, baseline_sample.heading_deg)
          speed_delta = abs(variant_sample.speed_ms - baseline_sample.speed_ms)
          if distance <= match_distance_m and heading_delta <= match_heading_deg and speed_delta <= match_speed_ms:
            score = distance / match_distance_m + heading_delta / match_heading_deg + speed_delta / match_speed_ms
            candidates.append((score, baseline_index, distance, heading_delta))
    if candidates:
      _, baseline_index, distance, heading_delta = min(candidates)
      speed_delta = abs(variant_sample.speed_ms - baseline.samples[baseline_index].speed_ms)
      matches.append((variant_index, baseline_index, distance, heading_delta, speed_delta))

  spatial_deltas: dict[tuple[int, int], list[float]] = defaultdict(list)
  match_distances: list[float] = []
  match_heading_deltas: list[float] = []
  match_speed_deltas: list[float] = []
  for variant_index, baseline_index, distance, heading_delta, speed_delta in matches:
    east, north = variant_xy[variant_index]
    spatial_key = (math.floor(east / spatial_bin_m), math.floor(north / spatial_bin_m))
    spatial_deltas[spatial_key].append(
      variant.samples[variant_index].ego_right_m - baseline.samples[baseline_index].ego_right_m
    )
    match_distances.append(distance)
    match_heading_deltas.append(heading_delta)
    match_speed_deltas.append(speed_delta)
  bin_deltas = np.asarray([np.median(values) for values in spatial_deltas.values()], dtype=float)
  details.update({
    "matched_sample_pairs": len(matches),
    "unique_baseline_samples_matched": len({match[1] for match in matches}),
    "spatial_bin_count": len(bin_deltas),
    "median_match_distance_m": _median(match_distances),
    "median_match_heading_delta_deg": _median(match_heading_deltas),
    "median_match_speed_delta_ms": _median(match_speed_deltas),
  })
  if not len(bin_deltas):
    return StageEvidence(
      "missing", f"only 0 independent spatial bins overlap; need {minimum_spatial_bins}", 0, details,
    )

  median_delta = float(np.median(bin_deltas))
  robust_spread = 1.4826 * float(np.median(np.abs(bin_deltas - median_delta)))
  confidence_low, confidence_high = _bootstrap_median_interval(bin_deltas)
  details.update({
    "median_rightward_delta_m": median_delta,
    "robust_spread_m": robust_spread,
    "median_bootstrap_ci95_m": [confidence_low, confidence_high],
    "spatial_bin_deltas_m": sorted(float(value) for value in bin_deltas),
    "fixed_bias_cancellation": (
      "A fixed camera/extrinsic lateral bias cancels in variant minus baseline, assuming the logged dongle's camera mount " +
      "and calibration did not change between runs."
    ),
  })
  if len(bin_deltas) < minimum_spatial_bins:
    return StageEvidence(
      "missing", f"only {len(bin_deltas)} independent spatial bins overlap; need {minimum_spatial_bins}",
      len(bin_deltas), details,
    )

  clearly_wrong = (
    confidence_high < 0.0
    or confidence_high < expected_delta_m - 1.5 * physical_tolerance_m
    or confidence_low > expected_delta_m + 1.5 * physical_tolerance_m
  )
  if clearly_wrong:
    return StageEvidence(
      "contradicted",
      f"matched A/B evidence gives {median_delta:+.3f} m, with sign or magnitude clearly inconsistent with {expected_delta_m:+.2f} m",
      len(bin_deltas), details,
    )
  confidence_width = confidence_high - confidence_low
  if (
    abs(median_delta - expected_delta_m) <= physical_tolerance_m
    and confidence_low > 0.0
    and robust_spread <= 0.08
    and confidence_width <= 0.10
  ):
    return StageEvidence(
      "proven",
      f"matched spatial A/B bins show {median_delta:+.3f} m vehicle-right displacement for +10 cm PathOffset",
      len(bin_deltas), details,
    )
  return StageEvidence(
    "missing",
    f"A/B median is {median_delta:+.3f} m, but uncertainty or spread is too large for a +0.10 m claim",
    len(bin_deltas), details,
  )


def analyze_events(events: Iterable[Any], *, source: str = "events", expected_path_offset_m: float = 0.10,
                   minimum_samples: int = 5, straight_curvature_limit: float = 0.001,
                   require_code_compatibility: bool = True,
                   mpc_replayer: Callable[[FrameEvidence], MpcReplayResult] | None = replay_mpc_static_counterfactual,
                   max_mpc_frames: int = 20, repo_root: Path | None = None) -> LaneOffsetReport:
  sorted_events = sorted(events, key=lambda event: int(_nested(event, "logMonoTime", 0)))
  records: dict[str, list[tuple[int, Any]]] = defaultdict(list)
  models: dict[int, Any] = {}
  params_epochs: list[tuple[int, dict[str, str]]] = []
  car_params_records: list[tuple[int, Any]] = []
  plan_events: list[Any] = []
  branch = ""
  commit = ""

  for event in sorted_events:
    try:
      name = _which(event)
    except Exception:
      continue
    mono_time = int(_nested(event, "logMonoTime", 0))
    payload = _payload(event, name)
    records[name].append((mono_time, payload))
    if name == "modelV2":
      models[mono_time] = payload
    elif name == "initData":
      params_epochs.append((mono_time, _params_from_init(payload)))
      branch = str(_nested(payload, "gitBranch", branch))
      commit = str(_nested(payload, "gitCommit", commit))
    elif name == "carParams":
      car_params_records.append((mono_time, payload))
    elif name == "lateralPlan":
      plan_events.append(event)

  if repo_root is None:
    repo_root = Path(__file__).resolve().parents[3]
  code_compatible = _code_compatible_with_checkout(commit, repo_root) if commit else None
  frames = [
    _make_frame(event, records, models, params_epochs, car_params_records, straight_curvature_limit)
    for event in plan_events
  ]
  straight_frames = [frame for frame in frames if frame.straight]
  exact_frames = [frame for frame in straight_frames if frame.geometry_exact and math.isfinite(frame.static_residual_m)]

  stages: dict[str, StageEvidence] = {}
  configured = [frame.configured_path_offset_m for frame in frames if math.isfinite(frame.configured_path_offset_m)]
  configured_match = [abs(value - expected_path_offset_m) <= 1e-9 for value in configured]
  if not params_epochs or not configured:
    stages["configured_path_offset"] = StageEvidence(
      "missing", "initData does not contain a logged PathOffset value", 0,
    )
  elif _ratio(configured_match) == 1.0:
    stages["configured_path_offset"] = StageEvidence(
      "proven", f"route-start Params record PathOffset={expected_path_offset_m:+.2f} m",
      len(configured), {"values_m": sorted(set(configured))},
    )
  else:
    stages["configured_path_offset"] = StageEvidence(
      "contradicted", f"PathOffset is not consistently {expected_path_offset_m:+.2f} m",
      len(configured), {"values_m": sorted(set(configured))},
    )

  dynamic_values = [frame.dynamic_offset_m for frame in straight_frames if math.isfinite(frame.dynamic_offset_m)]
  adjust_values = [frame.adjust_lane_offset_m for frame in frames if math.isfinite(frame.adjust_lane_offset_m)]
  direct_dynamic_count = sum(frame.evidence_source == "direct_numeric" for frame in straight_frames)
  if dynamic_values:
    stages["dynamic_adjust_lane_offset"] = StageEvidence(
      "proven",
      "numeric lateralPlan evidence exposes the applied dynamic offset" if direct_dynamic_count else
      "legacy latDebugText exposes the rounded dynamic offset for strict reconstruction",
      len(dynamic_values),
      {
        "configured_adjust_lane_offset_m": sorted(set(adjust_values)),
        "applied_dynamic_min_m": min(dynamic_values),
        "applied_dynamic_median_m": _median(dynamic_values),
        "applied_dynamic_max_m": max(dynamic_values),
        "direct_numeric_samples": direct_dynamic_count,
        "legacy_text_precision_m": 0.001,
      },
    )
  else:
    stages["dynamic_adjust_lane_offset"] = StageEvidence(
      "missing", "no straight lane-mode frame contains numeric dynamic evidence or a parseable legacy latDebugText offset", 0,
      {"configured_adjust_lane_offset_m": sorted(set(adjust_values))},
    )

  compatibility_allows_proof = code_compatible is True or not require_code_compatibility
  authoritative_frames = [
    frame for frame in exact_frames
    if frame.evidence_source == "direct_numeric" or compatibility_allows_proof
  ]
  residual_matches = [
    abs(frame.path_offset_m - expected_path_offset_m) <= STATIC_TOLERANCE_M
    and
    abs(frame.static_residual_m - expected_path_offset_m) <= STATIC_TOLERANCE_M
    and frame.static_max_error_m <= STATIC_TOLERANCE_M
    for frame in authoritative_frames
  ]
  if len(authoritative_frames) < minimum_samples:
    stages["lateral_plan_static_offset"] = StageEvidence(
      "missing",
      f"only {len(authoritative_frames)} authoritative straight lane-mode frames; need {minimum_samples}",
      len(authoritative_frames),
      {
        "direct_numeric_frames": sum(frame.evidence_source == "direct_numeric" for frame in exact_frames),
        "legacy_reconstruction_frames": sum(frame.evidence_source == "legacy_reconstruction" for frame in exact_frames),
        "straight_lane_mode_frames": len(straight_frames),
        "all_lateral_plan_frames": len(frames),
      },
    )
  elif _ratio(residual_matches) >= 0.8:
    stages["lateral_plan_static_offset"] = StageEvidence(
      "proven",
      f"{_ratio(residual_matches):.0%} of authoritative frames satisfy dPathPoints - pathBeforeStaticOffset = " +
      f"staticPathOffset = {expected_path_offset_m:+.2f} m",
      len(authoritative_frames),
      {
        "matching_ratio": _ratio(residual_matches),
        "median_static_residual_m": _median([frame.static_residual_m for frame in authoritative_frames]),
        "max_point_error_m": max(frame.static_max_error_m for frame in authoritative_frames),
        "direct_numeric_frames": sum(frame.evidence_source == "direct_numeric" for frame in authoritative_frames),
        "legacy_reconstruction_frames": sum(frame.evidence_source == "legacy_reconstruction" for frame in authoritative_frames),
        "tolerance_m": STATIC_TOLERANCE_M,
      },
    )
  else:
    stages["lateral_plan_static_offset"] = StageEvidence(
      "contradicted",
      f"only {_ratio(residual_matches):.0%} of authoritative frames contain the configured static residual",
      len(authoritative_frames),
      {
        "matching_ratio": _ratio(residual_matches),
        "median_static_residual_m": _median([frame.static_residual_m for frame in authoritative_frames]),
      },
    )

  mpc_candidates = [frame for frame, matches in zip(authoritative_frames, residual_matches, strict=True) if matches]
  replay_results: list[MpcReplayResult] = []
  if mpc_replayer is not None and compatibility_allows_proof:
    for frame in mpc_candidates[:max_mpc_frames]:
      replay_results.append(mpc_replayer(frame))
  usable_replays = [result for result in replay_results if result.usable]
  if stages["lateral_plan_static_offset"].status != "proven":
    stages["mpc_static_offset_effect"] = StageEvidence(
      "missing", "the static lateralPlan input must be proven before attributing an MPC effect", 0,
    )
  elif len(usable_replays) < minimum_samples:
    reasons = sorted({result.reason for result in replay_results if result.reason})
    stages["mpc_static_offset_effect"] = StageEvidence(
      "missing",
      f"only {len(usable_replays)} route-matched MPC counterfactuals; need {minimum_samples}",
      len(usable_replays),
      {
        "attempted": len(replay_results),
        "reasons": reasons[:5],
        "recorded_plan_early_curvature_median": _median([frame.plan_early_curvature for frame in mpc_candidates]),
      },
    )
  else:
    stages["mpc_static_offset_effect"] = StageEvidence(
      "proven",
      "removing only PathOffset in route-matched MPC counterfactuals removes a right-positive curvature contribution",
      len(usable_replays),
      {
        "median_observed_replay_rmse": _median([result.observed_rmse for result in usable_replays]),
        "median_right_curvature_delta": _median([result.right_curvature_delta for result in usable_replays]),
        "median_right_curvature_peak": _median([result.right_curvature_peak for result in usable_replays]),
      },
    )

  # controlsState carries modelV2 mono time, the same source id stored by
  # lateralPlan. Verify lane-plan selection numerically without requiring an
  # absolute steering sign: a working offset must eventually steer back while
  # holding its shifted position.
  control_candidates = mpc_candidates if compatibility_allows_proof else []
  candidate_by_model_time = {frame.model_mono_time: frame for frame in control_candidates}
  active_lane_line: list[bool] = []
  plan_errors: list[float] = []
  raw_model_errors: list[float] = []
  closer_to_plan: list[bool] = []
  linked_controls_times: list[tuple[int, int]] = []
  for mono_time, controls_state in records.get("controlsState", []):
    model_time = int(_nested(controls_state, "lateralPlanMonoTime", 0))
    frame = candidate_by_model_time.get(model_time)
    if frame is None:
      continue
    linked_controls_times.append((mono_time, model_time))
    active_lane_line.append(bool(_nested(controls_state, "activeLaneLine", False)))
    car_state = _latest_before(records.get("carState", []), mono_time, 200_000_000)
    live_delay = _latest_before(records.get("liveDelay", []), mono_time, 2_000_000_000)
    plan_target = _lag_adjusted_plan_curvature(frame, car_state, live_delay) if car_state is not None else NAN
    observed = float(_nested(controls_state, "desiredCurvature", NAN))
    raw_target = frame.model_desired_curvature
    if all(math.isfinite(value) for value in (plan_target, observed, raw_target)) and abs(plan_target - raw_target) > 1e-6:
      plan_error = abs(observed - plan_target)
      raw_error = abs(observed - raw_target)
      plan_errors.append(plan_error)
      raw_model_errors.append(raw_error)
      closer_to_plan.append(plan_error < raw_error)

  active_ratio = _ratio(active_lane_line)
  closer_ratio = _ratio(closer_to_plan)
  controls_details = {
    "linked_samples": len(linked_controls_times),
    "active_lane_line_ratio": active_ratio,
    "numerically_discriminating_samples": len(closer_to_plan),
    "closer_to_lane_plan_ratio": closer_ratio,
    "median_error_to_lane_plan": _median(plan_errors),
    "median_error_to_raw_model": _median(raw_model_errors),
  }
  if not compatibility_allows_proof and stages["lateral_plan_static_offset"].status == "proven":
    stages["controls_lane_plan_selection"] = StageEvidence(
      "missing", "route/current control sources differ, so the lag-adjusted control target cannot be reconstructed authoritatively",
      0, controls_details,
    )
  elif len(linked_controls_times) < minimum_samples:
    stages["controls_lane_plan_selection"] = StageEvidence(
      "missing", f"only {len(linked_controls_times)} controlsState messages link to proven lane plans", len(linked_controls_times),
      controls_details,
    )
  elif active_ratio < 0.8:
    stages["controls_lane_plan_selection"] = StageEvidence(
      "contradicted", f"controlsState.activeLaneLine is true in only {active_ratio:.0%} of linked samples",
      len(linked_controls_times), controls_details,
    )
  elif len(closer_to_plan) < minimum_samples:
    stages["controls_lane_plan_selection"] = StageEvidence(
      "missing", "lane-plan selection is flagged active, but logged targets do not differ enough from the raw model to discriminate",
      len(linked_controls_times), controls_details,
    )
  elif closer_ratio >= 0.7 and _median(plan_errors) < _median(raw_model_errors):
    stages["controls_lane_plan_selection"] = StageEvidence(
      "proven", "activeLaneLine is true and desiredCurvature follows the lag-adjusted lane plan more closely than raw model fallback",
      len(linked_controls_times), controls_details,
    )
  else:
    stages["controls_lane_plan_selection"] = StageEvidence(
      "missing", "activeLaneLine is true, but downsampled desiredCurvature does not numerically distinguish the filtered targets",
      len(linked_controls_times), controls_details,
    )

  torque_control = True
  if car_params_records:
    steer_type = _enum_name(_nested(car_params_records[-1][1], "steerControlType", "torque"))
    torque_control = steer_type != "angle"
  command_pairs: list[tuple[float, float, float]] = []
  for controls_time, _ in linked_controls_times:
    car_control_record = _latest_record_before(records.get("carControl", []), controls_time + 20_000_000, 40_000_000)
    car_output_record = _latest_record_before(records.get("carOutput", []), controls_time + 30_000_000, 50_000_000)
    if car_control_record is None or car_output_record is None or not bool(_nested(car_control_record[1], "latActive", False)):
      continue
    request_field = "actuators.torque" if torque_control else "actuators.steeringAngleDeg"
    applied_field = "actuatorsOutput.torque" if torque_control else "actuatorsOutput.steeringAngleDeg"
    can_field = "actuatorsOutput.torqueOutputCan" if torque_control else "actuatorsOutput.steeringAngleDeg"
    request = float(_nested(car_control_record[1], request_field, NAN))
    applied = float(_nested(car_output_record[1], applied_field, NAN))
    can_value = float(_nested(car_output_record[1], can_field, NAN))
    if all(math.isfinite(value) for value in (request, applied, can_value)):
      command_pairs.append((request, applied, can_value))

  nonzero_pairs = [pair for pair in command_pairs if abs(pair[0]) > 1e-4 or abs(pair[1]) > 1e-4]
  same_request_sign = [request * applied > 0.0 for request, applied, _ in nonzero_pairs]
  same_can_sign = [applied * can_value > 0.0 for _, applied, can_value in nonzero_pairs if abs(applied) > 1e-4 and abs(can_value) > 1e-4]
  command_details = {
    "paired_samples": len(command_pairs),
    "nonzero_samples": len(nonzero_pairs),
    "request_to_limited_output_sign_ratio": _ratio(same_request_sign),
    "limited_output_to_can_sign_ratio": _ratio(same_can_sign),
    "median_request_to_limited_abs_error": _median([abs(request - applied) for request, applied, _ in command_pairs]),
    "note": "This verifies CarController command propagation only, not physical steering direction or lane displacement.",
  }
  if len(nonzero_pairs) < minimum_samples:
    stages["carcontroller_command_tracking"] = StageEvidence(
      "missing", f"only {len(nonzero_pairs)} nonzero linked carControl/carOutput pairs", len(nonzero_pairs), command_details,
    )
  elif _ratio(same_request_sign) >= 0.8 and (not same_can_sign or _ratio(same_can_sign) >= 0.8):
    stages["carcontroller_command_tracking"] = StageEvidence(
      "proven", "CarController-limited output and CAN authority retain the requested command sign",
      len(nonzero_pairs), command_details,
    )
  else:
    stages["carcontroller_command_tracking"] = StageEvidence(
      "missing", "rate limiting/timestamp sampling prevents reliable request-to-output matching",
      len(nonzero_pairs), command_details,
    )

  stages["physical_rightward_displacement"] = StageEvidence(
    "missing",
    "a single +10 cm route cannot prove physical displacement; use a PathOffset=0/+10 A/B pair or a logged setting step with lane-center response",
    0,
  )

  observability_gaps = [
    "initData records Params when loggerd starts the route, not every mid-route setting change.",
    "carOutput proves the command surviving CarController limits, not the physical EPS/vehicle response or absolute lane position.",
    "An independent vehicle-to-lane offset or controlled PathOffset=0/+10 A/B run is needed to prove physical displacement.",
  ]
  if not any(frame.evidence_source == "direct_numeric" for frame in frames):
    observability_gaps.insert(
      1,
      "This route predates numeric staticPathOffset/dynamicLaneOffset/pathBeforeStaticOffset fields; legacy reconstruction depends " +
      "on model geometry and rounded latDebugText (1 mm precision).",
    )
  diagnostics = []
  if not plan_events:
    diagnostics.append("No lateralPlan messages were found; use rlog rather than a reduced log when possible.")
  if not params_epochs:
    diagnostics.append("No initData Params snapshot was found.")
  if code_compatible is False:
    diagnostics.append("The route commit has control-source differences from the current checkout.")
  if code_compatible is None and require_code_compatibility:
    diagnostics.append("The route commit could not be verified against the current checkout.")

  return LaneOffsetReport(
    source=source,
    expected_path_offset_m=expected_path_offset_m,
    branch=branch,
    commit=commit,
    code_compatible=code_compatible,
    stages=stages,
    frames=frames,
    observability_gaps=observability_gaps,
    diagnostics=diagnostics,
  )


def analyze_log(identifier: str, **kwargs: Any) -> LaneOffsetReport:
  from openpilot.tools.lib.logreader import LogReader
  return analyze_events(LogReader(identifier, sort_by_time=True), source=identifier, **kwargs)


def analyze_log_pair(baseline_identifier: str, variant_identifier: str, *,
                     minimum_spatial_bins: int = 8, match_distance_m: float = 8.0,
                     match_heading_deg: float = 7.0, match_speed_ms: float = 2.0,
                     spatial_bin_m: float = 20.0, physical_tolerance_m: float = 0.04,
                     dynamic_zero_tolerance_m: float = ZERO_DYNAMIC_TOLERANCE_M,
                     **kwargs: Any) -> LaneOffsetReport:
  from openpilot.tools.lib.logreader import LogReader

  baseline_events = list(LogReader(baseline_identifier, sort_by_time=True))
  variant_events = list(LogReader(variant_identifier, sort_by_time=True))
  report = analyze_events(variant_events, source=variant_identifier, **kwargs)
  report.stages["physical_rightward_displacement"] = analyze_physical_ab_events(
    baseline_events,
    variant_events,
    expected_delta_m=report.expected_path_offset_m,
    minimum_spatial_bins=minimum_spatial_bins,
    match_distance_m=match_distance_m,
    match_heading_deg=match_heading_deg,
    match_speed_ms=match_speed_ms,
    spatial_bin_m=spatial_bin_m,
    physical_tolerance_m=physical_tolerance_m,
    dynamic_zero_tolerance_m=dynamic_zero_tolerance_m,
  )
  report.stages["physical_rightward_displacement"].details.update({
    "baseline_source": baseline_identifier,
    "variant_source": variant_identifier,
  })
  if report.stages["physical_rightward_displacement"].status == "proven":
    report.observability_gaps = [
      gap for gap in report.observability_gaps
      if "PathOffset=0/+10" not in gap
    ]
  return report


def format_text(report: LaneOffsetReport) -> str:
  lines = [
    f"source: {report.source}",
    f"route code: {report.branch or '(unknown)'} {report.commit or '(unknown)'}",
    f"control-source compatible: {report.code_compatible}",
    f"expected PathOffset: {report.expected_path_offset_m:+.2f} m",
    "",
    "evidence stages:",
  ]
  for name, stage in report.stages.items():
    lines.append(f"  [{stage.status.upper():12}] {name}: {stage.summary} (n={stage.sample_count})")
    if stage.details:
      lines.append(f"    {json.dumps(_json_finite(stage.details), ensure_ascii=False, sort_keys=True)}")
  if report.diagnostics:
    lines.extend(("", "diagnostics:"))
    lines.extend(f"  - {item}" for item in report.diagnostics)
  lines.extend(("", "observability gaps:"))
  lines.extend(f"  - {item}" for item in report.observability_gaps)
  return "\n".join(lines)


def main() -> int:
  parser = argparse.ArgumentParser(
    description=__doc__,
    epilog="exit status: 0 core propagation proven, 2 contradicted, 3 insufficient evidence",
  )
  parser.add_argument("log", help="local rlog/qlog path, URL, or LogReader route identifier")
  parser.add_argument("--baseline-log",
                      help="PathOffset=0 validation rlog/route; positional log must be the matched +10 cm variant")
  parser.add_argument("--path-offset-cm", type=float, default=10.0, help="expected static PathOffset in cm (default: 10)")
  parser.add_argument("--minimum-samples", type=int, default=5)
  parser.add_argument("--max-mpc-frames", type=int, default=20)
  parser.add_argument("--allow-code-mismatch", action="store_true",
                      help="allow reconstruction to count as proof even if route/current control sources differ")
  parser.add_argument("--skip-mpc-replay", action="store_true")
  parser.add_argument("--minimum-spatial-bins", type=int, default=8)
  parser.add_argument("--match-distance-m", type=float, default=8.0)
  parser.add_argument("--match-heading-deg", type=float, default=7.0)
  parser.add_argument("--match-speed-ms", type=float, default=2.0)
  parser.add_argument("--spatial-bin-m", type=float, default=20.0)
  parser.add_argument("--physical-tolerance-cm", type=float, default=4.0)
  parser.add_argument("--dynamic-zero-tolerance-cm", type=float, default=0.5)
  parser.add_argument("--json", action="store_true", help="emit JSON")
  parser.add_argument("--include-frames", action="store_true", help="include per-frame evidence in JSON")
  args = parser.parse_args()
  common_arguments = {
    "expected_path_offset_m": args.path_offset_cm * 0.01,
    "minimum_samples": args.minimum_samples,
    "require_code_compatibility": not args.allow_code_mismatch,
    "mpc_replayer": None if args.skip_mpc_replay else replay_mpc_static_counterfactual,
    "max_mpc_frames": args.max_mpc_frames,
  }
  if args.baseline_log:
    report = analyze_log_pair(
      args.baseline_log,
      args.log,
      minimum_spatial_bins=args.minimum_spatial_bins,
      match_distance_m=args.match_distance_m,
      match_heading_deg=args.match_heading_deg,
      match_speed_ms=args.match_speed_ms,
      spatial_bin_m=args.spatial_bin_m,
      physical_tolerance_m=args.physical_tolerance_cm * 0.01,
      dynamic_zero_tolerance_m=args.dynamic_zero_tolerance_cm * 0.01,
      **common_arguments,
    )
  else:
    report = analyze_log(args.log, **common_arguments)
  if args.json:
    print(json.dumps(report.to_dict(include_frames=args.include_frames), ensure_ascii=False, indent=2, sort_keys=True))
  else:
    print(format_text(report))
  required_stages = (
    "configured_path_offset",
    "dynamic_adjust_lane_offset",
    "lateral_plan_static_offset",
    "mpc_static_offset_effect",
    "controls_lane_plan_selection",
    "carcontroller_command_tracking",
  )
  if args.baseline_log:
    required_stages += ("physical_rightward_displacement",)
  if any(report.stages[name].status == "contradicted" for name in required_stages):
    return 2
  if any(report.stages[name].status != "proven" for name in required_stages):
    return 3
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
