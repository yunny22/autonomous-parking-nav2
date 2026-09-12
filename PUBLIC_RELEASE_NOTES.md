# Public release staging notes

## Source selection

The staging repository was extracted from the `xycar_parking_nav` package in the
local Kookmin Gazebo workspace. Package history was inspected separately from the
parent workspace; the parent workspace and unrelated projects were not copied.
The source package had a local branch ahead of its remote, so this folder is a
reviewable snapshot rather than a claim that the upstream repository is public.

## Included

- ROS 2 Python package source and package metadata
- Nav2, mission, odometry, adapter and scan-filter configuration templates
- simulation, navigation and real-vehicle integration launch templates
- sequential mission manager and Ackermann behavior tree
- core algorithm and contract tests
- synthetic map for public tests and examples
- design and architecture documentation

## Excluded or sanitized

- competition PGM/YAML map and venue-specific goal coordinates
- measured vehicle calibration, private device paths and network addresses
- bags, recordings, datasets, model weights, build products and caches
- parent workspace files and unrelated team projects

The synthetic map is intentionally used as the default. The competition map should
only be added after redistribution permission is confirmed.

## Provenance and review items

The package history and core mission/validation logic provide evidence for a
personal development contribution. Nav2, ROS 2, and sensor/vehicle drivers remain
third-party dependencies and are not represented as newly implemented here.
Before a public GitHub release, confirm map ownership, team/source licensing, and
hardware revalidation on an authorized vehicle.

## Validation performed for staging

YAML/XML/Python syntax and unit tests are run from this clean folder. The ROS build
and launch checks depend on the host's installed ROS 2/Nav2 and external Xycar
message/driver packages; missing host dependencies are reported instead of being
vendored into this repository.
