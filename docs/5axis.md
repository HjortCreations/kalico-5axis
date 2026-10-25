# Kalico 5axis guide

This experimental Hjort Creations variant adds coordinated XYZAB motion to
CoreXY and Hybrid-CoreXY printers with exactly three independently driven Z
supports. A and B tilt the bed. Further development by Hjort Creations is
paused; this snapshot is provided for investigation and reuse.

## What the firmware controls

| Coordinate | Meaning |
| --- | --- |
| X, Y | Linear toolhead coordinates in the printer frame |
| Z in pivot mode | Bed-plane height at `pivot_position` |
| Z in TCP mode | Bed-plane height at the current X/Y position |
| A | Bed rotation about X, in degrees, following the right-hand rule |
| B | Bed rotation about Y, in degrees, following the right-hand rule |

The kinematics convert the requested bed pose into three support heights.
In TCP mode the supports also compensate for X/Y travel over a tilted plane.
Changing transform mode rebases logical Z to preserve the support positions.

This TCP implementation compensates bed-plane height. It does not apply a
complete rigid-body workpiece transformation to X/Y/Z. A toolpath generator
must account for the actual machine geometry, the moving workpiece and the
coordinate conventions used here.

For a pure A/B move, `F` is degrees per minute. For a move with linear travel,
`F` is millimeters per minute; configured rotary and support limits also
apply. A/B limits and the optional combined-tilt limit are separate settings.

## Hardware requirements

- A supported CoreXY or Hybrid-CoreXY gantry and exactly three independent
  Z stepper drives.
- Three measured support positions forming a triangle, mapped in
  `stepper_z`, `stepper_z1`, `stepper_z2` order.
- Supports and joints that permit tilt without binding, together with
  sufficient individual Z travel and mechanical clearance.
- Machine-specific endstops, homing, leveling, probe handling and accessible
  emergency stop.

A stock printer with three Z motors is not automatically compatible. Bed
mounts, guides, joints, cable routing, probes and toolhead geometry can limit
or prevent intentional tilt. Software motion limits do not detect collisions
with the machine or an existing print.

## Configuration and installation

Use the [CoreXY example](../config/example-corexy-3z-tilt.cfg) or
[Hybrid-CoreXY example](../config/example-hybrid-corexy-3z-tilt.cfg) as a
structural reference. They contain example pins and a 300 mm support layout;
they are not the configuration for the 200 mm motion demonstration.

1. Back up the existing firmware revision, configuration and working macros.
2. Review the changes in an isolated checkout of this repository before
   selecting it as the printer's firmware source. Follow the normal
   [Kalico installation procedure](https://docs.kalico.gg/Installation.html)
   with this repository as the source. Do not replace a working installation
   merely to try the example configuration.
3. Select `corexy_3z_tilt` or `hybrid_corexy_3z_tilt` under `[printer]`, with
   the corresponding tilt configuration section.
4. Set the actual pins, motor directions, travel, support geometry and pivot.
   Derive `max_a`, `max_b` and `max_combined_tilt` from the mechanical setup;
   configure angular and Z-support speed/acceleration limits for that setup.
5. Review homing and leveling macros. Z homing and direct leveling establish
   the level-bed reference. Validate this on the actual hardware before tilt.
6. Start with small, slow, unloaded motion, then validate each axis and
   transform separately before considering coordinated moves or extrusion.

See the [configuration reference](Config_Reference.md) for all parameters.
This repository does not provide a universal printer installation profile.

## Transform commands

| Command | Effect |
| --- | --- |
| `SET_TILT_TRANSFORM MODE=PIVOT` or `G49` | Select pivot-based Z |
| `SET_TILT_TRANSFORM MODE=TCP` or `G43.4` | Select bed-plane height compensation |
| `G1 ... A... B...` | Request bed tilt in the currently selected mode |

`transform_default` sets the initial mode. These aliases have the meanings
implemented in this variant; they do not imply every feature of CNC TCP.
See the [G-code reference](G-Codes.md#corexy_3z_tilt) for details.

## Bed mesh and other limitations

Bed mesh profiles do not track A/B or the transform mode. Before changing A
or B, run `BED_MESH_CLEAR`. If mesh compensation is needed, set the target
angles first, calibrate at those angles, and keep them unchanged while that
mesh is active. Do not reuse a saved mesh at different angles. Review start
macros for automatic profile loading as well.

The supplied motion dry run does not clear a mesh itself. Clear it manually
before that run. Clear or recalibrate it before relying on mesh compensation
after transform changes as well.

There is no slicer, path planner for nonplanar deposition, automatic
collision detection or extrusion-process validation in this project.
Ordinary planar G-code does not become a five-axis print by enabling TCP or
adding arbitrary tilt commands. Probe workflows and other add-ons must be
checked individually. Do not assume upstream support for those combinations.

## Existing demonstration and validation

The [motion dry run](../dry_runs/README.md) targets a modified RatRig
V-Core 3.1 with a 200 mm build volume and its own stated limits. It commands
the heaters off, requests no extrusion and expects the machine to be
homed already. It changes modal state and velocity settings. Its final
velocity command repeats its chosen values instead of restoring previous
settings. Read the complete file and instructions before use.

For filming, an unloaded motion demonstration can show the X/Y/Z travel,
A/B tilt and support compensation. Identify motion-only footage as such.
Successful extrusion, surface quality, strength and support reduction
require separate print evidence.

Existing automated coverage includes:

- [CoreXY unit tests](../test/test_corexy_3z_tilt.py) for support geometry,
  sign conventions, transform behavior and motion limits.
- [Hybrid-CoreXY unit tests](../test/test_hybrid_corexy_3z_tilt.py) for
  position reconstruction and TCP behavior.
- [CoreXY G-code test](../test/klippy/corexy_3z_tilt.test) and
  [Hybrid-CoreXY G-code test](../test/klippy/hybrid_corexy_3z_tilt.test) for
  homing, coordinated moves and transform commands in the test environment.

The repository's Build test workflow runs the existing automated suite.
Its results establish software test outcomes, not physical machine safety
or print performance. Keep real-machine reports separate and include the
revision, configuration, geometry, commands and observed result.

## Reuse and maintenance

This is an independent variant of Kalico/Klipper, not an official upstream
or RatRig release. Preserve the repository's license and attribution notices.
Issues and contributions can document findings, but there is no commitment
to review, merge or provide support while Hjort Creations development is
paused. No automatic monthly release schedule is intended for this variant.
