# Public Release Notes

## Source revision

- Original package: `xycar_kookmin_gazebo_track/xycar_ws/src/xycar_parking_nav`
- Original repository remote: `yunny22/kookmin_sim_to_real`
- Source branch: `빠킹`
- Selected source revision: `856b24f0c39fde0368fd7c98171fab2aa96d8eec`

The source branch was five commits ahead of its remote-tracking branch when this
public release was audited. The parent workspace and unrelated projects
were not copied; the original workspace remains unchanged.

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

## Provenance, credits and license status

The selected package history records the mission, validation, parking
configuration and odometry integration under one local author identity. Team
permission to publish the retained project code is confirmed. Nav2, ROS 2, and
sensor/vehicle drivers remain third-party dependencies and are not represented
as newly implemented here.

No project LICENSE is included. Package metadata is `UNLICENSED` because this
portfolio repository grants no separate open-source reuse license. Third-party
dependency terms remain in force. The competition map remains excluded, so its
venue/map redistribution right is not required for this public tree.

## Validation performed for public release

YAML/XML/Python syntax and unit tests are run from this clean folder. The ROS build
and launch checks depend on the host's installed ROS 2/Nav2 and external Xycar
message/driver packages; missing host dependencies are reported instead of being
vendored into this repository.
