import pytest

from openpilot.selfdrive.controls.lib.lane_planner_2 import LANE_WIDTH_ASYMMETRY_DEADBAND_M, lane_width_adjust_offset


ADJUST_LANE_OFFSET_M = 0.10
DEADBAND_EPSILON_M = 1e-6


@pytest.mark.parametrize(("left_width", "right_width", "expected"), [
  (2.1, 2.1, 0.0),
  (2.2, 2.2, 0.0),
  (2.3, 2.3, 0.0),
  (1.9, 1.9, 0.0),
  (2.2, 2.1, 0.1),
  (2.1, 2.2, -0.1),
])
def test_lane_width_adjust_offset_has_no_left_bias_on_equal_widths(left_width, right_width, expected):
  assert lane_width_adjust_offset(2.9, left_width, right_width, ADJUST_LANE_OFFSET_M) == pytest.approx(expected)


def test_lane_width_adjust_offset_fades_out_on_narrow_lane():
  assert lane_width_adjust_offset(2.5, 2.2, 2.1, ADJUST_LANE_OFFSET_M) == pytest.approx(0.0)


@pytest.mark.parametrize("direction", [-1.0, 1.0], ids=["right-side-wider", "left-side-wider"])
@pytest.mark.parametrize("difference_m", [
  LANE_WIDTH_ASYMMETRY_DEADBAND_M / 10.0,
  LANE_WIDTH_ASYMMETRY_DEADBAND_M,
], ids=["small-noise", "deadband-boundary"])
def test_lane_width_adjust_offset_ignores_near_equal_widths_in_both_directions(direction, difference_m):
  left_width = 2.1 + max(direction * difference_m, 0.0)
  right_width = 2.1 + max(-direction * difference_m, 0.0)

  assert lane_width_adjust_offset(2.9, left_width, right_width, ADJUST_LANE_OFFSET_M) == pytest.approx(0.0)


@pytest.mark.parametrize(("direction", "expected"), [
  (1.0, ADJUST_LANE_OFFSET_M),
  (-1.0, -ADJUST_LANE_OFFSET_M),
], ids=["left-side-wider-moves-right", "right-side-wider-moves-left"])
def test_lane_width_adjust_offset_preserves_clear_asymmetry_in_both_directions(direction, expected):
  difference_m = LANE_WIDTH_ASYMMETRY_DEADBAND_M + DEADBAND_EPSILON_M
  left_width = 2.1 + max(direction * difference_m, 0.0)
  right_width = 2.1 + max(-direction * difference_m, 0.0)

  assert lane_width_adjust_offset(2.9, left_width, right_width, ADJUST_LANE_OFFSET_M) == pytest.approx(expected)


def test_lane_width_noise_cannot_cancel_static_right_offset_but_clear_asymmetry_can():
  static_path_offset = ADJUST_LANE_OFFSET_M
  noise_dynamic_offset = lane_width_adjust_offset(
    2.9,
    2.1,
    2.1 + LANE_WIDTH_ASYMMETRY_DEADBAND_M,
    ADJUST_LANE_OFFSET_M,
  )
  clear_dynamic_offset = lane_width_adjust_offset(
    2.9,
    2.1,
    2.1 + LANE_WIDTH_ASYMMETRY_DEADBAND_M + DEADBAND_EPSILON_M,
    ADJUST_LANE_OFFSET_M,
  )

  assert noise_dynamic_offset == pytest.approx(0.0)
  assert static_path_offset + noise_dynamic_offset == pytest.approx(ADJUST_LANE_OFFSET_M)
  assert clear_dynamic_offset == pytest.approx(-ADJUST_LANE_OFFSET_M)
  assert static_path_offset + clear_dynamic_offset == pytest.approx(0.0)
