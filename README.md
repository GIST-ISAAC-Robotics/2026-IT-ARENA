# 2026 IT ARENA 자율주행 경주

GIST-ISAAC-Robotics의 차량 소프트웨어·ROS 2 시뮬레이션 저장소입니다. 현재 개발 구성은 B0183 RGB + C1 LiDAR + 엔코더/자이로 운동 보정 + 지역 Pure Pursuit입니다.

- [현재 구성·검증 상태·다음 작업](docs/PROJECT_CONTEXT.md)
- [코스 개편 전 보관 릴리스·복원 방법](docs/releases/PRE_TRACK_REDESIGN_2026_11_05.md)
- [현행 개발 계획](docs/DEVELOPMENT_PLAN.md) · [하드웨어 설계 전달·규정 질문](docs/HARDWARE_AND_RULE_QUESTIONS.md)
- [작업별 문서 안내와 기록 방식](docs/README.md)
- [기여·에이전트 작업 규칙](AGENTS.md)
- [주최 측 공식 저장소](https://github.com/MOSW626/istech-it-arena)

## 시작하기

PC 기준 환경은 **WSL2 Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic**입니다. 최초 설치에는 `sudo bash scripts/setup_wsl.sh`와 `bash scripts/configure_wsl_user.sh`를 사용합니다. 기존 환경에서는 다음을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/doctor.sh
colcon build --symlink-install --base-paths src
source install/setup.bash
python3 scripts/run_current_solo.py outputs/jr_solo_01
```

최종 CAD 전 **신규 시뮬 개발의 임시 차량은 JR60P20**입니다([ADR 0029](docs/decisions/0029-temporary-jr60p20-vehicle.md)). 위 명령은 [JR60P20 센서 설정](src/arena_description/config/b0183_c1_jr60p20_sim.yaml)(뒤 지주 바깥 배선 예약 외곽 포함)과 높인 뒤 마커, 구조 가림 처리, 기존 단독 속도 프로필·가상 MCU로 1랩을 실행합니다. 이 구성의 제한 단독·교통 주행 결과와 남은 문제는 [현재 현황](docs/PROJECT_CONTEXT.md)에 있습니다. `--straight-budget`으로 기본 OFF 직선 속도 후보를 선택할 수 있습니다. 매 실행에 새로운 결과 경로를 지정합니다. `--print-args`는 실행 없이 조합을 확인하고 `--previous-jr50`은 이전 JR50을 재현합니다. 교통 실행 인자는 [채택 보고서](artifacts/validation/2026-11-04/lidar_jr60_adoption_v1/README.md#4-앞으로의-실행-인자)에 있습니다.

기존 범용 예제는 `ros2 launch arena_bringup local_pursuit.launch.py`입니다. 이 예제의 기본 `local_fast` 명령 상한은 9 km/h이며 구동 연결은 `legacy`입니다. 기존 비교 재현과 새 단독 시뮬의 설정을 구분합니다. [속도 목표](docs/decisions/0028-speed-continuity-and-final-gearing.md) · [주행 설명](docs/autonomy/LOCAL_PURE_PURSUIT_2026_09_21.md) · [구동 계약/ROS 연결](docs/hardware/ACTUATION_ROS_2026_09_22.md)

`headless:=true`는 GUI만 끕니다. WSL GPU 비교에는 `render_backend:=wsl_nvidia`를 명시하고 실제 렌더러·정상 종료를 확인합니다. 기본 렌더러는 `system`입니다. [설정·문제 해결](docs/simulation/GPU_RENDERING.md)

## 저장소 구성

| 경로 | 역할 |
|---|---|
| `src/arena_autonomy/` | 인식·운동 보정·지역 경로·주행·독립 보호층 |
| `src/arena_vehicle_interface/` | 센서/구동 어댑터·명령 계약·가상 MCU·계측 |
| `src/arena_bringup/` | 실행 조합과 launch 설정 |
| `src/arena_description/` | 차량 치수·센서 배치·모델 |
| `src/arena_gazebo/` | 월드·구동 물리·신호등 |
| `scripts/`, `tests/`, `config/` | 생성/실험/분석 도구, 회귀검사, 시험 설정 |
| `docs/` | 현재 현황·설계·결정·짧은 활동 기록; 과거 상세는 `docs/archive/` |
| `assets/`, `artifacts/`, `outputs/` | 공식 원본, 검증 증거, 예산표/조사 산출물 |

## 기존 실행과 검증

`simulation.launch.py`는 시뮬레이션만, `demo.launch.py`는 기존 LiDAR/스테레오 비교 구성을 실행합니다. 한 번에 하나씩 실행하고 종료를 확인합니다. 트랙은 기본 `official`, 비교용 `experimental`/`original`을 선택할 수 있습니다. [기존 데모](docs/autonomy/BASIC_DEMO.md)

공식 실행 입력은 `config/tracks/official_v2026.11.01.yaml`입니다. 공식 방지턱 제거·검정 그리드·2×5 신호·ID0/45 배치를 반영한 팀 파생본이며, 사용자 확인에 따라 코스 안 신호등 기둥은 제거했습니다. [공식 입력과 팀 변경 구분](docs/reports/OFFICIAL_UPDATE_2026_11_02.md)·[기둥 제거 검증](artifacts/validation/2026-11-03/post_free_facility_v1/README.md). 원본 ZIP과 기존 시험은 보존합니다. 변경 영향에 맞게 검증 명령을 선택하십시오.

```bash
python3 scripts/build_official_track.py --check
python3 -m pytest tests src/arena_vehicle_interface/test -q
```

`--check`는 출처/해시 검사이며 실제 주행 검증이 아닙니다. 전체 회귀를 실행하지 않은 변경에 과거 통과 결과를 새 결과로 사용하지 않습니다. 현재 실패·미검증 범위는 [현황](docs/PROJECT_CONTEXT.md), 이전 실행 예시 전체는 [정리 전 README](docs/archive/2026-09-27/README.before.md)에 있습니다.
