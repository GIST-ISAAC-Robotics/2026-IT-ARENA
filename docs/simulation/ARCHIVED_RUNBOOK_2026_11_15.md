# 보존된 개발 환경 실행 안내

2026-11-15 전시용 README 정리 때 기존 실행 설명을 옮겼다. 명령은 과거 개발 환경 재현용이며 고정 ESP32 최종 소스와 현재 `main`의 동일성을 보장하지 않는다. [최종 소스 복원](../releases/FINAL_ESP32_2026_11_14.md)과 [시험별 이력](../reports/SIMULATION_WORK_INDEX_2026_11_14.md)을 먼저 확인한다.

## 시작하기

PC 기준 환경은 **WSL2 Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic**입니다. 최초 설치에는 `sudo bash scripts/setup_wsl.sh`와 `bash scripts/configure_wsl_user.sh`를 사용합니다. 기존 환경에서는 다음을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/doctor.sh
colcon build --symlink-install --base-paths src
source install/setup.bash
python3 scripts/build_official_track.py --check --profile config/tracks/official_v2026.11.06.yaml
python3 scripts/run_new_course_solo.py outputs/new_course_solo_01 --preset new-course-solo --check-r25
python3 scripts/run_new_course_solo.py outputs/new_course_solo_01 --preset new-course-solo
```

- **새 코스 입력 점검:** 위 `--check --profile …v2026.11.06.yaml`이 새 공식 원본·팀 실행 월드(`track:=official_v2026_11_06`)의 출처/해시를 확인합니다. 프로필 없는 `--check`는 이전 v2026.11.01 입력만 확인합니다. 둘 다 주행 검증은 아닙니다. [새 코스 감사](../../docs/track/OFFICIAL_V2026_11_06_AUDIT.md)·[이관 근거](../../artifacts/validation/2026-11-07/track_migration_v1/README.md)
- **단독 본선 프리셋 `new-course-solo`:** B0390/C1·[JR60P20](../../docs/decisions/0029-temporary-jr60p20-vehicle.md)(ID49 중심 140 mm), 새 코스, 기본 OFF 단독 후보(직선 증속·직각 접근 감속·사선 공백·직각 벽 경로·선호 벽 재획득)를 묶어 1랩을 실행합니다. 이 조합은 [r25](../../artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/README.md)(거리 한 랩 46.67 m·29.228초)의 등록 실행 인자와 같습니다. `--check-r25`는 실행 없이 경로만 정상화해 각 옵션·값을 대조하고, `--print-args`는 최종 인자만 출력합니다. 인자 일치일 뿐, 현재 소스·기록 계측·실행 결과가 r25와 같다는 뜻은 아닙니다. STOP 정착 변위 등 남은 한계는 [현황](../../docs/PROJECT_CONTEXT.md)에 있습니다.
- **결과 경로:** 매 실행에 새 경로를 지정합니다(기존 경로는 거절). 기존 `artifacts/validation/` 결과 아래에 쓰지 않습니다.
- **교통:** 교통 시험은 단독 프리셋에 포함하지 않으며 별도 프로필을 씁니다. 현재 대표와 한계는 [교통 경기 거리 대표](../../artifacts/validation/2026-11-08/traffic_race_distance_v1/README.md)·[교통 한 랩 r23](../../artifacts/validation/2026-11-08/traffic_lap_transition_v1/README.md)과 [현황](../../docs/PROJECT_CONTEXT.md)을 확인합니다.
- **이전 구성 재현:** `python3 scripts/run_current_solo.py <새 경로>`(옵션 생략)는 이전 v2026.11.01 코스·[B0183 센서 설정](../../src/arena_description/config/b0183_c1_jr60p20_sim.yaml)의 기존 단독 조합이며, `--previous-jr50`은 이전 JR50을 재현합니다. 이전 코스 교통 인자는 [채택 보고서](../../artifacts/validation/2026-11-04/lidar_jr60_adoption_v1/README.md#4-앞으로의-실행-인자)에 있습니다.

기존 범용 예제는 `ros2 launch arena_bringup local_pursuit.launch.py`입니다. 이 예제의 기본 `local_fast` 명령 상한은 9 km/h이며 구동 연결은 `legacy`입니다. 기존 비교 재현과 새 단독 시뮬의 설정을 구분합니다. [속도 목표](../../docs/decisions/0028-speed-continuity-and-final-gearing.md) · [주행 설명](../../docs/autonomy/LOCAL_PURE_PURSUIT_2026_09_21.md) · [구동 계약/ROS 연결](../../docs/hardware/ACTUATION_ROS_2026_09_22.md)

`headless:=true`는 GUI만 끕니다. WSL GPU 비교에는 `render_backend:=wsl_nvidia`를 명시하고 실제 렌더러·정상 종료를 확인합니다. 기본 렌더러는 `system`입니다. [설정·문제 해결](../../docs/simulation/GPU_RENDERING.md)

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

`simulation.launch.py`는 시뮬레이션만, `demo.launch.py`는 기존 LiDAR/스테레오 비교 구성을 실행합니다. 한 번에 하나씩 실행하고 종료를 확인합니다. 트랙은 기본 `official`, 비교용 `experimental`/`original`을 선택할 수 있습니다. [기존 데모](../../docs/autonomy/BASIC_DEMO.md)

신규 시험의 공식 실행 입력은 `config/tracks/official_v2026.11.06.yaml`(월드 `track:=official_v2026_11_06`)입니다. 이전 `config/tracks/official_v2026.11.01.yaml`(`track:=official`, 방지턱 제거·검정 그리드·2×5 신호·ID0/45 배치·코스 안 신호등 기둥 제거 팀 파생본)은 기존 시험 재현용입니다. [이전 입력과 팀 변경 구분](../../docs/reports/OFFICIAL_UPDATE_2026_11_02.md)·[기둥 제거 검증](../../artifacts/validation/2026-11-03/post_free_facility_v1/README.md). 원본 ZIP과 기존 시험은 보존합니다. 변경 영향에 맞게 검증 명령을 선택하십시오.

```bash
python3 scripts/build_official_track.py --check --profile config/tracks/official_v2026.11.06.yaml
python3 scripts/build_official_track.py --check   # 이전 v2026.11.01 재현 입력
python3 -m pytest tests src/arena_vehicle_interface/test -q
```

`--check`는 출처/해시 검사이며 실제 주행 검증이 아닙니다. 전체 회귀를 실행하지 않은 변경에 과거 통과 결과를 새 결과로 사용하지 않습니다. 현재 실패·미검증 범위는 [현황](../../docs/PROJECT_CONTEXT.md), 이전 실행 예시 전체는 [정리 전 README](../../docs/archive/2026-09-27/README.before.md)에 있습니다.
