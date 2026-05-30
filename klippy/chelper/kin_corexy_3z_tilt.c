// Three-point tilting bed kinematics for CoreXY printers
//
// Copyright (C) 2026  Kalico contributors
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // cos, tan
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

#define DEGREES_TO_RADIANS (M_PI / 180.)

struct corexy_3z_tilt_stepper {
    struct stepper_kinematics sk;
    double support_x, support_y;
    double pivot_x, pivot_y;
};

static double
corexy_3z_tilt_stepper_calc_position(struct stepper_kinematics *sk,
                                     struct move *m, double move_time)
{
    struct corexy_3z_tilt_stepper *ts = container_of(
        sk, struct corexy_3z_tilt_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double a = c.a * DEGREES_TO_RADIANS;
    double b = c.b * DEGREES_TO_RADIANS;
    double dx = ts->support_x - ts->pivot_x;
    double dy = ts->support_y - ts->pivot_y;
    return c.z + dy * tan(a) / cos(b) - dx * tan(b);
}

struct stepper_kinematics * __visible
corexy_3z_tilt_stepper_alloc(double support_x, double support_y,
                             double pivot_x, double pivot_y)
{
    struct corexy_3z_tilt_stepper *ts = malloc(sizeof(*ts));
    memset(ts, 0, sizeof(*ts));
    ts->support_x = support_x;
    ts->support_y = support_y;
    ts->pivot_x = pivot_x;
    ts->pivot_y = pivot_y;
    ts->sk.calc_position_cb = corexy_3z_tilt_stepper_calc_position;
    ts->sk.active_flags = AF_Z | AF_A | AF_B;
    return &ts->sk;
}
