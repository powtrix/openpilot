import pytest

from openpilot.selfdrive.controls.lib.lane_planner_2 import lane_width_adjust_offset


@pytest.mark.parametrize(("left_width", "right_width", "expected"), [
  (2.1, 2.1, 0.0),
  (2.2, 2.2, 0.0),
  (2.3, 2.3, 0.0),
  (1.9, 1.9, 0.0),
  (2.2, 2.1, 0.1),
  (2.1, 2.2, -0.1),
])
def test_lane_width_adjust_offset_has_no_left_bias_on_equal_widths(left_width, right_width, expected):
  assert lane_width_adjust_offset(2.9, left_width, right_width, 0.1) == pytest.approx(expected)


def test_lane_width_adjust_offset_fades_out_on_narrow_lane():
  assert lane_width_adjust_offset(2.5, 2.2, 2.1, 0.1) == pytest.approx(0.0)
