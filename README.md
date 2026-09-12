# Kookmin Autonomous Parking (Nav2)

국민대학교 자율주차 대회를 위해 제공된 정적 지도를 사용해 waypoint 기반
주차 미션을 구성한 ROS 2 패키지다. 이 저장소는 온라인 SLAM을 구현하는 예제가
아니라, 미리 준비한 지도와 주행 중 위치 추정을 결합하는 자율주차 구조를
공개용으로 정리한 것이다.

## 시스템 흐름

```text
pre-built static map
  -> 31 mission goal poses
  -> mission manager sends one pose at a time
  -> Nav2 NavigateToPose
  -> SmacPlannerHybrid (Reeds–Shepp)
  -> MPPIController (Ackermann)
  -> vehicle command adapter
```

31개 항목은 주차·전환·복귀를 위한 목표 pose다. 연속 trajectory를 직접 만들어
실행한 것이 아니며, mission manager가 각 목표를 Nav2에 순서대로 전달하고
waypoint 사이의 실제 경로는 Nav2가 생성한다.

## Localization

VESC 이동거리와 IMU gyro yaw를 이용해 odometry를 만들고, 2D LiDAR scan과
미리 준비한 정적 지도를 AMCL에 입력해 주행 중 위치를 보정한다. 대회 지도를
주행 중 새로 만드는 SLAM 패키지는 포함하지 않는다.

## Nav2 구성

- Planner: `nav2_smac_planner::SmacPlannerHybrid`, `REEDS_SHEPP`
- Controller: `nav2_mppi_controller::MPPIController`, `Ackermann`
- Mission: sequential `NavigateToPose` action client
- Behavior tree: Ackermann 경로 추종에 필요한 재계획·복구 흐름
- Adapter: Nav2 `cmd_vel`을 차량 인터페이스로 전달하고, 기본적으로 shadow 모드 사용

설정 파일의 차량 치수·센서 보정값은 재사용 가능한 공개 템플릿으로 정리했다.
실제 대회 지도, 좌표, 장치 경로와 비공개 calibration 값은 포함하지 않았다.

## 공개용 시뮬레이션

`maps/example_map.yaml`과 `example_map.pgm`은 패키지 구조와 오프라인 검증을
보여주기 위한 합성 예제 지도다. 실제 대회 지도는 사용 권한이 확인될 때까지
공개 저장소 후보에서 제외했다. `parking_sim.launch.py`는 이 합성 지도를
사용하며 실차 모터 토픽으로 출력하지 않는다.

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths . --packages-select xycar_parking_nav
source install/setup.bash

python3 -m pytest -q test
ros2 run xycar_parking_nav validate_map
ros2 launch xycar_parking_nav parking_sim.launch.py
```

실차 launch는 통합 구조를 설명하기 위한 템플릿이다. 기본값은
`drive_enabled:=false`, 센서 드라이버 실행과 preflight도 비활성화되어 있으며,
사용자는 자신의 장치·드라이버·지도 경로를 명시적으로 연결해야 한다.

## 실제 검증에서 확인한 한계

Gazebo와 별도의 임시 시험 환경에서는 waypoint 기반 주행을 확인했지만, 실제
대회장에서는 IMU 오차와 LiDAR–map 정합 불안정으로 AMCL Localization이 흔들렸다.
현장 센서와 주행 파라미터를 충분히 다시 보정할 시간이 부족해 시험 환경의
주행을 안정적으로 재현하지 못했다. 이 저장소는 해당 경험을 숨기지 않고,
실제 현장 재검증이 필요한 공개 후보로 제시한다.

## 구조

```text
config/           Nav2, mission, odometry, adapter 공개 템플릿
launch/           simulation / navigation / real integration launch
behavior_trees/   Ackermann NavigateToPose behavior tree
maps/             합성 예제 지도와 provenance 안내
xycar_parking_nav/mission manager, localization, adapter, 검증 로직
test/             알고리즘·설정 계약 테스트
docs/             설계와 공개 범위 기록
```

`xycar_msgs`, VESC·LiDAR 드라이버, Nav2는 별도 ROS 2 의존성이다. 이 패키지의
mission·검증 로직은 개인 개발 범위에 해당하지만, 외부 드라이버와 Nav2 구현을
새로 작성한 것으로 주장하지 않는다.
