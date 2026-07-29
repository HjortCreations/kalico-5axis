# RatRig V-Core 3.1 200 mm — 5-axis motion dry run

This directory contains a full-motion G-code dry run for a **modified RatRig
V-Core 3.1 with a 200 mm build volume** using Kalico's `corexy_3z_tilt`
kinematics.

> [!CAUTION]
> Run this file entirely at your own risk. It moves every motion axis and is
> not a universal RatRig test. It is not endorsed by RatRig. A configuration,
> origin, travel limit, tilt limit, or tool-offset mismatch can cause a
> collision or damage the machine.

## File

- [`5axis_full_motion_dry_run.gcode`](5axis_full_motion_dry_run.gcode) tests
  individual and coordinated X, Y, Z, A, and B motion in pivot and TCP modes.
- Heaters are commanded off and no extrusion is requested.
- Homing is intentionally **not** performed by the file.

## Required configuration

The file was prepared for these limits:

- X: 0–200 mm
- Y: 0–200 mm
- Z: -5–200 mm
- A and B: ±6 degrees individually
- Combined bed tilt: maximum 6 degrees

The printer must use the matching 5-axis Kalico `corexy_3z_tilt`
configuration and support `G43.4` (TCP mode) and `G49` (pivot mode).

## Before running

1. Read the complete G-code and compare every coordinate, feed rate, and
   velocity limit with your own `printer.cfg`.
2. Home all axes manually and confirm that the printer reports the expected
   coordinates.
3. Remove any workpiece, print, loose tool, or other obstruction from the
   full motion envelope, and stow detachable probes if your setup requires it.
4. Confirm that the bed supports, cables, and toolhead can safely complete
   the commanded tilt and travel.
5. Be ready to trigger the emergency stop throughout the run.

The test begins near the center of the assumed 200 × 200 mm work area, runs
conservative XYZ and tilt patterns, tests TCP motion, and returns to a neutral
centered position.
