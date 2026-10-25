# Kalico 5axis

Experimental XYZAB motion control for CoreXY and Hybrid-CoreXY printers with a three-point tilting bed, by Hjort Creations. Built on [Kalico](https://github.com/KalicoCrew/kalico), a community fork of [Klipper](https://github.com/Klipper3d/klipper).

Three independently driven Z supports control bed height and two tilt angles. This project adds coordinated `X Y Z A B` moves and a selectable bed-plane compensation mode to explore that motion on suitable hardware.

> **Experimental firmware snapshot.** This is a motion-control foundation for developers and experienced machine builders. It does not include a five-axis slicer or an automatic collision-free printing workflow. Hardware and print performance must be validated for each machine.

## Project status

Further development by Hjort Creations is paused. This work is shared as open source so that other developers, researchers and machine builders can explore it and decide whether to take it further. There is no promised release schedule, ongoing support or slicer development commitment.

Existing tests and a machine-specific motion dry run are included. Automated checks exercise software behavior; they do not certify a printer or demonstrate successful five-axis extrusion.

## What is included

- `corexy_3z_tilt` and `hybrid_corexy_3z_tilt` kinematics for exactly three independently driven Z supports.
- A and B tilt commands in `G0`/`G1`, with axis, combined-tilt and Z-support motion limits configurable for the machine.
- **Pivot mode:** Z describes bed height at a configured virtual pivot.
- **TCP mode:** Z describes tilted bed-plane height at the current X/Y location; the Z supports compensate as X/Y changes.
- Example configurations, configuration and G-code references, regression tests, and an existing motion-only demonstration for a modified RatRig V-Core 3.1 200 mm.

TCP here means bed-plane height compensation. It is not a general tool-center-point transformation of an entire 3D workpiece, and does not turn ordinary slicer output into five-axis toolpaths.

## Start here

| Resource | Purpose |
| --- | --- |
| [5-axis guide](docs/5axis.md) | Hardware requirements, coordinate conventions, setup, limitations and validation |
| [CoreXY example](config/example-corexy-3z-tilt.cfg) | Configuration structure; adapt all pins, geometry and limits |
| [Hybrid-CoreXY example](config/example-hybrid-corexy-3z-tilt.cfg) | Configuration structure for a hybrid gantry |
| [Configuration reference](docs/Config_Reference.md#corexy-kinematics-with-three-point-bed-tilt) | Detailed CoreXY parameters; Hybrid-CoreXY has its own section in the same reference |
| [G-code reference](docs/G-Codes.md#corexy_3z_tilt) | Transform commands and bed-mesh restrictions |
| [Motion dry run](dry_runs/README.md) | Existing machine-specific demonstration, without heating or extrusion |
| [Contributing](CONTRIBUTING.md) | Reuse, issue reports and the project's paused maintenance status |

Read the guide before installing or running anything. A printer having three Z motors is not enough: its supports, joints, guides, cables and toolhead clearance must permit intentional bed tilt.

## Current boundaries

- No five-axis slicer, toolpath generator, collision simulation or automatic orientation planning is included.
- Bed mesh is not angle-aware. Clear it **before** changing A/B; a mesh calibrated at one angle must not be reused at another.
- Example configurations are not ready-to-install printer profiles. The dry run targets a different, specific geometry and limit set.
- Machine-specific homing, leveling, probes, macros and other extensions require separate validation.
- Potential printing benefits such as fewer supports, smoother surfaces or stronger parts remain application-dependent goals, not demonstrated results of this snapshot.

## Credits and license

The 5-axis additions build on the work of the Kalico and Klipper communities. This repository is an independent Hjort Creations variant; it is not an official Kalico, Klipper or RatRig release.

The repository retains the GNU GPLv3 license in [COPYING](COPYING), existing copyright notices and third-party license files. Keep those notices with redistributed code and consult the applicable license files for individual components.

General Kalico information is available in the [upstream documentation](https://docs.kalico.gg/). The local 5-axis guide and references describe this variant's additions.
