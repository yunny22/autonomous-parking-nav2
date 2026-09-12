# 공개 설계 개요

이 패키지는 국민대학교 자율주차 대회를 위해 작성한 정적 지도 기반 Nav2
통합 예제다. 대회에서 제공한 지도 위에 주행 목표 pose를 직접 정의하고,
Nav2가 목표 사이의 경로 생성과 Ackermann 차량 추종을 담당하도록 구성했다.

## 미션과 경로

`parking_mission.yaml`의 31개 항목은 연속 궤적이 아니라 주행·주차·복귀를
나누는 목표 pose다. mission manager는 각 목표를 `NavigateToPose` action으로
하나씩 전송한다. planner는 `SmacPlannerHybrid`의 Reeds–Shepp 운동 모델을
사용하고, controller는 Ackermann MPPI로 경로를 추종한다.

## 위치 추정

VESC 이동거리와 IMU gyro yaw를 조합해 `slam_odom`을 만들고, 2D LiDAR scan과
정적 지도에 대한 AMCL 결과로 위치를 보정한다. 주행 중 지도를 새로 만드는
online SLAM은 이 패키지의 범위가 아니다.

## 검증 구조

시뮬레이터는 합성 PGM 지도에서 명령을 kinematic pose로 적분하고 LiDAR scan을
ray-cast해 Nav2 graph를 점검한다. adapter는 기본적으로 shadow 명령만 내보내며,
실차 launch도 `drive_enabled=false`와 드라이버 비활성값을 기본으로 사용한다.
하드웨어 검증을 할 때는 사용자의 센서 드라이버, 장치 경로, 허가된 지도와
차량별 calibration을 별도 인자로 연결해야 한다.

## 실제 환경의 한계

Gazebo와 별도 임시 시험 환경에서는 waypoint 주행을 확인했지만, 실제 대회장에서는
IMU 오차와 LiDAR–map 정합 불안정으로 localization이 흔들렸다. 현장에 맞춘
센서·지도·주행 파라미터 보정 시간을 충분히 확보하지 못해 시험 환경의 동작을
안정적으로 재현하지 못했다. 따라서 이 공개 예제는 실차 성공을 주장하지 않으며,
실제 차량 적용 전 반복 검증이 필요하다.
