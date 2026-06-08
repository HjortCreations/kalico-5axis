import pytest

from klippy.kinematics.hybrid_corexy_3z_tilt import (
    HybridCoreXY3ZTiltKinematics,
)


class DummyRail:
    def __init__(self, name):
        self.name = name

    def get_name(self):
        return self.name


class DummyStepper:
    def __init__(self, name):
        self.name = name

    def get_name(self):
        return self.name


def make_kinematics(inverted=False):
    kin = object.__new__(HybridCoreXY3ZTiltKinematics)
    kin.rails = [DummyRail("stepper_x"), DummyRail("stepper_y")]
    kin.z_steppers = [
        DummyStepper("stepper_z"),
        DummyStepper("stepper_z1"),
        DummyStepper("stepper_z2"),
    ]
    kin.z_positions = ((0.0, 0.0), (300.0, 0.0), (150.0, 300.0))
    kin.pivot_position = (150.0, 150.0)
    (x0, y0), (x1, y1), (x2, y2) = kin.z_positions
    kin.support_determinant = (
        (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    )
    kin.inverted = inverted
    return kin


def test_calc_position_matches_hybrid_corexy():
    kin = make_kinematics()
    position = [0.0, 0.0, 120.0, 0.0, 3.0, -2.0]
    h0, h1, h2 = kin._get_support_heights(position)
    stepper_positions = {
        "stepper_x": 30.0,
        "stepper_y": 10.0,
        "stepper_z": h0,
        "stepper_z1": h1,
        "stepper_z2": h2,
    }
    assert kin.calc_position(stepper_positions) == pytest.approx(
        [40.0, 10.0, 120.0, 3.0, -2.0]
    )


def test_calc_position_matches_inverted_hybrid_corexy():
    kin = make_kinematics(inverted=True)
    position = [0.0, 0.0, 80.0, 0.0, -1.0, 2.0]
    h0, h1, h2 = kin._get_support_heights(position)
    stepper_positions = {
        "stepper_x": 30.0,
        "stepper_y": 10.0,
        "stepper_z": h0,
        "stepper_z1": h1,
        "stepper_z2": h2,
    }
    assert kin.calc_position(stepper_positions) == pytest.approx(
        [20.0, 10.0, 80.0, -1.0, 2.0]
    )
