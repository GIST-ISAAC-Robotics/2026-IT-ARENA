# 2026-10-02 모터축 엔코더 기준선 반영: 정적·단위·모델 생성 검사

작업: Claude. Gazebo·ROS launch·노드 프로세스 실행·센서 bag 재생·Jetson 접속은 하지 않았다. 이 자료는 시뮬레이션 주행·실차 검증이 아니다.

## 최종 판정

- `run_checks.sh v6` → `verdict_v6.txt`: **VERDICT=PASS**
  - 대상 회귀 339 passed (`pytest_targeted_v6.log`)
  - 넓은 회귀 542 passed (`pytest_broad_v6.log`). 실제 rclpy 노드나 하위 프로세스를 띄우는 5개 파일은 제외했다: `test_lidar_shutdown`, `test_local_pursuit`, `test_replay_imu_probe`, `test_replay_observer`, `test_wsl_d3d12_lifetime`
  - C++ 방정식 단위 검사는 저장소 밖 임시 경로에서 빌드·실행해 PASS (`cpp_v6.log`)
  - `arena_gazebo` 플러그인은 임시 build/install 경로에서 컴파일만 성공했고 실행하지 않았다 (`colcon_arena_gazebo_v6.log`)
- 이전 실행본은 덮어쓰지 않고 보존한다.
  - v1: WSL 인용 문제로 ROS 환경이 깨진 수집 오류
  - v2–v5: 수정 과정의 실패
  - v1–v4에는 종합 판정 기능이 없었다

## 외형 회귀에서 확인한 실제 한계

`geometry_diagnostic_v6.json`의 실험 트랙(공식 트랙 아님) 결과다.

- 직진 외형 0.200×0.150 m는 통과했다.
- 최대 조향 바퀴 bbox 201.098×174.078 mm는 `static_footprint_checks_pass=false`다.
  - 장애물·경로 표본 충돌은 0이다.
  - 엇갈린 출발칸끼리 (0,3)(1,4)(2,5) 쌍이 겹친다. 출발칸 간격은 가로 약 168.7 mm, 세로 약 201.2 mm이고 약 0.007 rad 기울어져 있다.
- 이전 바퀴(약 200.0×167.6 mm)에서는 통과했다.
- 트랙·출발칸·차량을 줄이지 않았다. `tests/test_track_variants.py`는 장애물 충돌과 출발칸 상호 겹침을 나눠 검사하고, 겹침 쌍을 그대로 고정해 기록한다.

## 공식 팀 파생 실행 월드

`regen_official_world.sh` → `official_world_regen_verdict.txt`

- `build_official_track.py` 생성과 `--check` 결과는 모두 exit 0이다.
- 바뀐 파일은 `provenance.json` 하나이며, `vehicle_config` 입력 해시만 갱신됐다.
- `world.sdf` 바이트는 동일하다(sha256 `f73b59d2…42af`).
- `assets/`는 변경되지 않았다.
- 이후 `vehicle.yaml`을 다시 바꾸면 이 해시도 다시 낡는다.
