# G1 recent-observation ROS fault validation v1

2026-10-17 재개 절차에 따른 빌드 및 ROS fault validator 단회 실행 기록입니다. 기존 소스·테스트·설정을 수정하지 않고, 실행 전후 hash를 보존합니다.

## 실행 범위

- WSL Ubuntu 24.04에서 ROS Jazzy와 `install/local_setup.bash`를 source합니다.
- 실행 셸의 기존 `PYTHONPATH` 뒤에 checkout의 ROS Python 경로를 추가합니다.
- `colcon build --symlink-install --packages-select arena_autonomy arena_bringup`를 실행합니다.
- 빌드 후 setup을 다시 적용하고 `validate_pipeline_faults_ros.py`를 `g1_recent_observation_candidate`, `full`, 600초 상한으로 한 번 실행합니다.
- 자동 재시도는 없습니다. 실패 결과도 그대로 보존합니다.

## 산출물

`run/`에 환경, 빌드, validator 출력, 실행 전후 `src/scripts/tests/config` hash 목록과 비교 결과를 기록합니다. 결과 해석 시 validator의 실제 정지, cleanup, 잔존 프로세스, runtime source hash 및 성공/실패 사례를 구분합니다. 이 검증은 실차, Jetson, 전원, Gazebo 주행 검증이 아닙니다.

## 실행

```bash
bash artifacts/validation/2026-10-17/g1_recent_observation_ros_v1/run.sh
```
