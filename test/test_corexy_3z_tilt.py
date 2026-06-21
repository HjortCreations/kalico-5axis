import math

import pytest

from klippy.kinematics.corexy_3z_tilt import CoreXY3ZTiltKinematics
from klippy.toolhead import Move


def make_kinematics():
    kin = object.__new__(CoreXY3ZTiltKinematics)
    kin.z_positions = ((0.0, 0.0), (300.0, 0.0), (150.0, 300.0))
    kin.pivot_position = (150.0, 150.0)
    (x0, y0), (x1, y1), (x2, y2) = kin.z_positions
    kin.support_determinant = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    kin.transform_enabled = False
    return kin


def test_support_height_pose_round_trip():
    kin = make_kinematics()
    position = [0.0, 0.0, 120.0, 0.0, 3.0, -2.0]
    heights = kin._get_support_heights(position)
    z, a, b = kin._get_bed_pose(heights)
    assert (z, a, b) == pytest.approx((120.0, 3.0, -2.0))


def test_positive_a_lifts_positive_y_side():
    kin = make_kinematics()
    heights = kin._get_support_heights([0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    assert heights[2] > heights[0]
    assert heights[2] > heights[1]


def test_positive_b_lowers_positive_x_side():
    kin = make_kinematics()
    heights = kin._get_support_heights([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    assert heights[0] > heights[1]


def test_direct_z_adjust_establishes_level_reference():
    kin = make_kinematics()
    position = [0.0, 0.0, 120.0, 0.0, 2.0, -1.0]
    adjusted = kin.get_z_reference_position(position)
    assert adjusted == [0.0, 0.0, 120.0, 0.0, 0.0, 0.0]
    assert position == [0.0, 0.0, 120.0, 0.0, 2.0, -1.0]


def test_tcp_transform_keeps_surface_height_at_tool_xy():
    kin = make_kinematics()
    kin.transform_enabled = True
    position = [200.0, 240.0, 120.0, 0.0, 3.0, -2.0]
    heights = kin._get_support_heights(position)
    pivot_z, a, b = kin._get_bed_pose(heights)
    reconstructed = [position[0], position[1], pivot_z, 0.0, a, b]
    surface_z = pivot_z + kin._surface_offset(reconstructed)
    assert surface_z == pytest.approx(position[2])


def test_tcp_xy_move_shifts_all_supports_equally():
    kin = make_kinematics()
    kin.transform_enabled = True
    start = [150.0, 150.0, 120.0, 0.0, 3.0, 0.0]
    end = [150.0, 250.0, 120.0, 0.0, 3.0, 0.0]
    deltas = [
        end_height - start_height
        for start_height, end_height in zip(
            kin._get_support_heights(start), kin._get_support_heights(end)
        )
    ]
    assert deltas[0] == pytest.approx(deltas[1])
    assert deltas[1] == pytest.approx(deltas[2])
    assert deltas[0] == pytest.approx(-100.0 * math.tan(math.radians(3.0)))


def test_transform_rebase_preserves_support_positions():
    kin = make_kinematics()
    pivot_position = [200.0, 240.0, 120.0, 0.0, 3.0, -2.0]
    pivot_heights = kin._get_support_heights(pivot_position)
    tcp_position = list(pivot_position)
    tcp_position[2] += kin._surface_offset(pivot_position)
    kin.transform_enabled = True
    assert kin._get_support_heights(tcp_position) == pytest.approx(
        pivot_heights
    )


def test_combined_tilt_exceeds_individual_axis_angle():
    kin = make_kinematics()
    assert kin._combined_tilt(5.0, 5.0) == pytest.approx(7.0666, abs=0.001)


class DummyToolhead:
    max_accel = 100.0
    max_accel_to_decel = 50.0
    max_velocity = 100.0
    junction_deviation = 0.01


def test_pure_a_move_is_kinematic():
    move = Move(
        DummyToolhead(),
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 2.0, 0.0],
        speed=1.0,
    )
    assert move.is_kinematic_move
    assert move.move_d == pytest.approx(2.0)
    assert move.axes_r[4] == pytest.approx(1.0)


def test_a_move_is_limited_by_support_motor_speed():
    kin = make_kinematics()
    kin.max_a = kin.max_b = 5.0
    kin.max_z_velocity = 5.0
    kin.max_z_accel = 30.0
    kin.max_a_velocity = kin.max_b_velocity = 10.0
    kin.max_a_accel = kin.max_b_accel = 100.0
    move = Move(
        DummyToolhead(),
        [0.0, 0.0, 120.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 120.0, 0.0, 2.0, 0.0],
        speed=10.0,
    )
    for support_position in kin.z_positions:
        kin._limit_support_speed(move, support_position)
    assert move.max_cruise_v2**0.5 < 2.0
