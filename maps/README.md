# Maps

`example_map.pgm` is a small synthetic map for public tests and launch examples.
It is not the competition map. The original PGM/YAML and venue-specific mission
coordinates were excluded because permission to redistribute them has not been
confirmed.

To use a permitted map locally, pass its YAML explicitly to the launch files:

```bash
ros2 launch xycar_parking_nav parking_navigation.launch.py map:=/path/to/map.yaml
```
