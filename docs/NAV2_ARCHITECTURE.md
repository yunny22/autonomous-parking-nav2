# Navigation graph

```text
static map -> map_server -> AMCL
VESC distance + IMU gyro -> odometry

mission manager -> NavigateToPose -> SmacPlannerHybrid (Reeds–Shepp)
                                     -> MPPIController (Ackermann)
                                     -> cmd_vel adapter -> vehicle interface
```

The mission manager owns sequencing and localization readiness. Nav2 owns path
generation and path following. The adapter provides a fail-closed command boundary
and publishes a shadow command when hardware output is disabled.
