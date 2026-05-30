# CoreXY kinematics with a three-point tilting bed
#
# Copyright (C) 2026  Kalico contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

from klippy import stepper


class CoreXY3ZTiltKinematics:
    def __init__(self, toolhead, config):
        self.rails = [
            stepper.LookupMultiRail(config.getsection("stepper_" + n))
            for n in "xyz"
        ]
        self.z_steppers = self.rails[2].get_steppers()
        if len(self.z_steppers) != 3:
            raise config.error(
                "corexy_3z_tilt requires exactly three Z steppers"
            )

        tilt_config = config.getsection("corexy_3z_tilt")
        self.z_positions = tilt_config.getlists(
            "z_positions", seps=(",", "\n"), parser=float, count=2
        )
        if len(self.z_positions) != 3:
            raise config.error(
                "corexy_3z_tilt z_positions requires exactly three items"
            )
        default_pivot = tuple(
            sum(position[i] for position in self.z_positions) / 3.0
            for i in (0, 1)
        )
        self.pivot_position = tilt_config.getfloatlist(
            "pivot_position", default_pivot, count=2
        )
        self._validate_support_triangle(config)

        # Setup axis rails
        for s in self.rails[1].get_steppers():
            self.rails[0].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)
        self.rails[0].setup_itersolve("corexy_stepper_alloc", b"+")
        self.rails[1].setup_itersolve("corexy_stepper_alloc", b"-")
        pivot_x, pivot_y = self.pivot_position
        for s, (support_x, support_y) in zip(
            self.z_steppers, self.z_positions
        ):
            s.setup_itersolve(
                "corexy_3z_tilt_stepper_alloc",
                support_x,
                support_y,
                pivot_x,
                pivot_y,
            )
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )

        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", max_velocity, above=0.0, maxval=max_velocity
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", max_accel, above=0.0, maxval=max_accel
        )
        self.max_a = tilt_config.getfloat(
            "max_a", 5.0, above=0.0, maxval=45.0
        )
        self.max_b = tilt_config.getfloat(
            "max_b", 5.0, above=0.0, maxval=45.0
        )
        self.max_a_velocity = tilt_config.getfloat(
            "max_a_velocity", 5.0, above=0.0
        )
        self.max_b_velocity = tilt_config.getfloat(
            "max_b_velocity", 5.0, above=0.0
        )
        self.max_a_accel = tilt_config.getfloat(
            "max_a_accel", 5.0, above=0.0
        )
        self.max_b_accel = tilt_config.getfloat(
            "max_b_accel", 5.0, above=0.0
        )
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

    def _validate_support_triangle(self, config):
        (x0, y0), (x1, y1), (x2, y2) = self.z_positions
        self.support_determinant = (
            (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        )
        if abs(self.support_determinant) < 0.000001:
            raise config.error(
                "corexy_3z_tilt z_positions must form a triangle"
            )

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def _get_support_heights(self, position):
        pivot_x, pivot_y = self.pivot_position
        a = math.radians(position[4])
        b = math.radians(position[5])
        return [
            position[2]
            + (support_y - pivot_y) * math.tan(a) / math.cos(b)
            - (support_x - pivot_x) * math.tan(b)
            for support_x, support_y in self.z_positions
        ]

    def _get_bed_pose(self, heights):
        (x0, y0), (x1, y1), (x2, y2) = self.z_positions
        pivot_x, pivot_y = self.pivot_position
        h0, h1, h2 = heights
        determinant = self.support_determinant
        slope_x = (
            (h1 - h0) * (y2 - y0) - (h2 - h0) * (y1 - y0)
        ) / determinant
        slope_y = (
            (x1 - x0) * (h2 - h0) - (x2 - x0) * (h1 - h0)
        ) / determinant
        z = h0 - slope_x * (x0 - pivot_x) - slope_y * (y0 - pivot_y)
        b = math.atan(-slope_x)
        a = math.atan(slope_y * math.cos(b))
        return z, math.degrees(a), math.degrees(b)

    def calc_position(self, stepper_positions):
        xy_pos = [
            stepper_positions[self.rails[i].get_name()] for i in (0, 1)
        ]
        heights = [stepper_positions[s.get_name()] for s in self.z_steppers]
        z, a, b = self._get_bed_pose(heights)
        return [
            0.5 * (xy_pos[0] + xy_pos[1]),
            0.5 * (xy_pos[0] - xy_pos[1]),
            z,
            a,
            b,
        ]

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        for axis in homing_axes:
            self.limits[axis] = self.rails[axis].get_range()

    def note_z_not_homed(self):
        self.clear_homing_state([2])

    def get_z_reference_position(self, position):
        # Z homing and direct leveling establish a new level bed reference.
        position = list(position)
        position[4] = position[5] = 0.0
        return position

    def clear_homing_state(self, axes):
        for i, _ in enumerate(self.limits):
            if i in axes:
                self.limits[i] = (1.0, -1.0)

    def home(self, homing_state):
        # Each linear axis is homed independently and in order.
        for axis in homing_state.get_axes():
            rail = self.rails[axis]
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None] * 6
            homepos[axis] = hi.position_endstop
            if axis == 2:
                # A simultaneous three-motor Z home defines a level bed.
                homepos[4] = homepos[5] = 0.0
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            homing_state.home_rails([rail], forcepos, homepos)

    def _motor_off(self, print_time):
        self.clear_homing_state((0, 1, 2))

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if move.axes_d[i] and (
                end_pos[i] < self.limits[i][0]
                or end_pos[i] > self.limits[i][1]
            ):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def _check_support_range(self, move):
        position_min, position_max = self.rails[2].get_range()
        start_heights = self._get_support_heights(move.start_pos)
        end_heights = self._get_support_heights(move.end_pos)
        if any(
            height < position_min or height > position_max
            for height in end_heights
        ):
            raise move.move_error("Z support height out of range")
        if any(
            height < position_min or height > position_max
            for height in start_heights
        ):
            # Homing deliberately starts outside the configured range.
            return
        max_a = math.radians(
            max(abs(move.start_pos[4]), abs(move.end_pos[4]))
        )
        max_b = math.radians(
            max(abs(move.start_pos[5]), abs(move.end_pos[5]))
        )
        min_z = min(move.start_pos[2], move.end_pos[2])
        max_z = max(move.start_pos[2], move.end_pos[2])
        pivot_x, pivot_y = self.pivot_position
        for support_x, support_y in self.z_positions:
            dx = support_x - pivot_x
            dy = support_y - pivot_y
            offset_limit = (
                abs(dy) * math.tan(max_a) / math.cos(max_b)
                + abs(dx) * math.tan(max_b)
            )
            if (
                min_z - offset_limit < position_min
                or max_z + offset_limit > position_max
            ):
                raise move.move_error("Z support height out of range")

    def _limit_axis_speed(self, move, axis, max_velocity, max_accel):
        axis_delta = abs(move.axes_d[axis])
        if axis_delta:
            ratio = move.move_d / axis_delta
            move.limit_speed(max_velocity * ratio, max_accel * ratio)

    def _get_max_support_derivatives(self, move, support_position):
        pivot_x, pivot_y = self.pivot_position
        support_x, support_y = support_position
        dx = abs(support_x - pivot_x)
        dy = abs(support_y - pivot_y)
        max_a = math.radians(self.max_a)
        max_b = math.radians(self.max_b)
        a_ratio = abs(math.radians(move.axes_r[4]))
        b_ratio = abs(math.radians(move.axes_r[5]))
        z_ratio = abs(move.axes_r[2])
        tan_a = math.tan(max_a)
        tan_b = math.tan(max_b)
        sec_a = 1.0 / math.cos(max_a)
        sec_b = 1.0 / math.cos(max_b)
        slope = (
            z_ratio
            + dy
            * (
                sec_a**2 * sec_b * a_ratio
                + tan_a * sec_b * tan_b * b_ratio
            )
            + dx * sec_b**2 * b_ratio
        )
        curvature = (
            dy
            * (
                2.0 * sec_a**2 * tan_a * sec_b * a_ratio**2
                + 2.0 * sec_a**2 * sec_b * tan_b * a_ratio * b_ratio
                + tan_a
                * sec_b
                * (tan_b**2 + sec_b**2)
                * b_ratio**2
            )
            + dx * 2.0 * sec_b**2 * tan_b * b_ratio**2
        )
        return slope, curvature

    def _limit_support_speed(self, move, support_position):
        slope, curvature = self._get_max_support_derivatives(
            move, support_position
        )
        if not slope:
            return
        max_velocity = self.max_z_velocity / slope
        max_accel = self.max_z_accel / slope
        if curvature:
            # Reserve half of the motor acceleration budget for curvature.
            max_velocity = min(
                max_velocity, math.sqrt(0.5 * self.max_z_accel / curvature)
            )
            max_accel *= 0.5
        move.limit_speed(max_velocity, max_accel)

    def check_move(self, move):
        xpos, ypos = move.end_pos[:2]
        if (
            xpos < self.limits[0][0]
            or xpos > self.limits[0][1]
            or ypos < self.limits[1][0]
            or ypos > self.limits[1][1]
        ):
            self._check_endstops(move)

        a, b = move.end_pos[4:6]
        if abs(a) > self.max_a or abs(b) > self.max_b:
            raise move.move_error("Bed tilt angle out of range")
        if not any(move.axes_d[i] for i in (2, 4, 5)):
            return
        if self.limits[2][0] > self.limits[2][1]:
            raise move.move_error("Must home Z axis before tilting bed")
        self._check_endstops(move)
        self._check_support_range(move)

        self._limit_axis_speed(
            move, 4, self.max_a_velocity, self.max_a_accel
        )
        self._limit_axis_speed(
            move, 5, self.max_b_velocity, self.max_b_accel
        )
        for support_position in self.z_positions:
            self._limit_support_speed(move, support_position)

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            "homed_axes": "".join(axes),
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
            "pivot_position": self.pivot_position,
            "z_positions": self.z_positions,
        }


def load_kinematics(toolhead, config):
    return CoreXY3ZTiltKinematics(toolhead, config)
