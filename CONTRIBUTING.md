# Contributing to Kalico 5axis

Hjort Creations development on this experimental variant is paused. The
existing work is being prepared for public investigation and reuse. There
is no promised response time, roadmap, release cadence or slicer project.

You are welcome to study the code, report reproducible findings and develop
your own fork under the applicable licenses. A pull request may be useful
to other readers even if it is not reviewed or merged here.

## Reporting a finding

For a 5-axis issue, include the commit, kinematics type, relevant sanitized
configuration, measured support geometry, transform mode, minimal commands,
expected result and actual result. State whether the result comes from a
software test, unloaded hardware motion or an extrusion test. Remove
credentials, network details and unrelated personal or customer information
from logs before posting.

Use this repository for findings specific to these additions. Do not ask
Kalico, Klipper or RatRig maintainers to support this independent variant.
Consult the [5-axis guide](docs/5axis.md) before attempting a reproduction on
hardware; do not execute someone else's machine-specific G-code unchanged.

## Proposing a change

Explain the concrete problem, coordinate assumptions, affected hardware and
validation. Extend existing regression coverage when behavior changes. Keep
motion-only demonstrations distinct from verified print results. Preserve
existing copyright notices and the licenses in [COPYING](COPYING) and the
third-party components.

Documentation and reusable test evidence are helpful contributions. Neither
an issue nor a pull request creates a maintenance commitment for this project.
