import math

import pytest

from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_effective_cruise_speed
from openpilot.selfdrive.controls.lib.longitudinal_planner import sanitize_v_cruise_kph


@pytest.mark.parametrize(
  ("raw_speed", "expected_speed", "expected_initialized"),
  (
    (100.0, 100.0, True),
    (V_CRUISE_MAX + 5.0, V_CRUISE_MAX, True),
    (V_CRUISE_UNSET, V_CRUISE_MAX, False),
    (-1.0, 0.0, False),
    (math.nan, 0.0, False),
    (math.inf, 0.0, False),
    (-math.inf, 0.0, False),
  ),
)
def test_sanitize_v_cruise_kph(raw_speed, expected_speed, expected_initialized):
  speed, initialized = sanitize_v_cruise_kph(raw_speed)
  assert speed == expected_speed
  assert initialized is expected_initialized


@pytest.mark.parametrize(
  ("planner_speed", "carrot_speed", "expected_speed"),
  (
    (0.0, 20.0, 0.0),
    (20.0, 3.0, 3.0),
    (10.0, 10.0, 10.0),
  ),
)
def test_effective_cruise_speed_uses_more_conservative_limit(planner_speed, carrot_speed, expected_speed):
  assert get_effective_cruise_speed(planner_speed, carrot_speed) == expected_speed
