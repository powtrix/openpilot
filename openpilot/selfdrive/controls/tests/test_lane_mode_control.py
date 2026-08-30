import pytest

from openpilot.selfdrive.controls.controlsd import lane_mode_control_enabled


@pytest.mark.parametrize(("use_lane_lines", "v_turn_speed", "curve_speed_threshold", "expected"), [
  (True, 0, 0, True),
  (True, 0, 80, True),
  (True, 81, 80, True),
  (True, -81, 80, True),
  (True, 80, 80, False),
  (True, 79, 80, False),
  (False, 0, 80, False),
  (False, 81, 80, False),
])
def test_lane_mode_control_gate(use_lane_lines, v_turn_speed, curve_speed_threshold, expected):
  assert lane_mode_control_enabled(use_lane_lines, v_turn_speed, curve_speed_threshold) is expected
