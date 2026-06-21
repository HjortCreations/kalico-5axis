# Hybrid-CoreXY kinematics with a three-point tilting bed
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from klippy import stepper

from .corexy_3z_tilt import CoreXY3ZTiltKinematics


class HybridCoreXY3ZTiltKinematics(CoreXY3ZTiltKinematics):
    def __init__(self, toolhead, config):
        self.rails = [
            stepper.LookupMultiRail(config.getsection("stepper_" + n))
            for n in "xyz"
        ]
        self.z_steppers = self.rails[2].get_steppers()
        if len(self.z_steppers) != 3:
            raise config.error(
                "hybrid_corexy_3z_tilt requires exactly three Z steppers"
            )

        tilt_config = config.getsection("hybrid_corexy_3z_tilt")
        self.z_positions = tilt_config.getlists(
            "z_positions", seps=(",", "\n"), parser=float, count=2
        )
        if len(self.z_positions) != 3:
            raise config.error(
                "hybrid_corexy_3z_tilt z_positions requires exactly three items"
            )
        default_pivot = tuple(
            sum(position[i] for position in self.z_positions) / 3.0
            for i in (0, 1)
        )
        self.pivot_position = tilt_config.getfloatlist(
            "pivot_position", default_pivot, count=2
        )
        self.transform_enabled = tilt_config.getchoice(
            "transform_default", {"pivot": False, "tcp": True}, "pivot"
        )
        self._validate_support_triangle(config)

        self.inverted = config.getboolean("invert_kinematics", False)
        self.corexy_mode = (b"-", b"+")
        if self.inverted:
            self.corexy_mode = (b"+", b"-")

        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)
        for i, s in enumerate(self.rails[0].get_steppers()):
            mode = self.corexy_mode[0] if i == 0 else self.corexy_mode[1]
            s.setup_itersolve("corexy_stepper_alloc", mode)
        self.rails[1].setup_itersolve("cartesian_stepper_alloc", b"y")

        pivot_x, pivot_y = self.pivot_position
        self.z_stepper_kinematics = []
        for s, (support_x, support_y) in zip(self.z_steppers, self.z_positions):
            s.setup_itersolve(
                "corexy_3z_tilt_stepper_alloc",
                support_x,
                support_y,
                pivot_x,
                pivot_y,
                self.transform_enabled,
            )
            self.z_stepper_kinematics.append(s.get_stepper_kinematics())
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )

        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", max_velocity, above=0.0, maxval=max_velocity
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", max_accel, above=0.0, maxval=max_accel
        )
        self.max_a = tilt_config.getfloat("max_a", 5.0, above=0.0, maxval=45.0)
        self.max_b = tilt_config.getfloat("max_b", 5.0, above=0.0, maxval=45.0)
        self.max_combined_tilt = tilt_config.getfloat(
            "max_combined_tilt", None, above=0.0, maxval=45.0
        )
        self.max_a_velocity = tilt_config.getfloat(
            "max_a_velocity", 5.0, above=0.0
        )
        self.max_b_velocity = tilt_config.getfloat(
            "max_b_velocity", 5.0, above=0.0
        )
        self.max_a_accel = tilt_config.getfloat("max_a_accel", 5.0, above=0.0)
        self.max_b_accel = tilt_config.getfloat("max_b_accel", 5.0, above=0.0)
        self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(
            ranges[0][0],
            ranges[1][0],
            ranges[2][0],
            0.0,
            -self.max_a,
            -self.max_b,
        )
        self.axes_max = toolhead.Coord(
            ranges[0][1],
            ranges[1][1],
            ranges[2][1],
            0.0,
            self.max_a,
            self.max_b,
        )
        self.supports_dual_carriage = False
        self.supports_rotary_axes = True
        self.toolhead = toolhead
        self._register_transform_commands(config.get_printer())

    def calc_position(self, stepper_positions):
        x_rail_pos = stepper_positions[self.rails[0].get_name()]
        y_pos = stepper_positions[self.rails[1].get_name()]
        heights = [stepper_positions[s.get_name()] for s in self.z_steppers]
        z, a, b = self._get_bed_pose(heights)
        if self.inverted:
            x_pos = x_rail_pos - y_pos
        else:
            x_pos = x_rail_pos + y_pos
        position = [x_pos, y_pos, z, 0.0, a, b]
        if self.transform_enabled:
            position[2] += self._surface_offset(position)
        return position[:3] + position[4:]


def load_kinematics(toolhead, config):
    return HybridCoreXY3ZTiltKinematics(toolhead, config)
