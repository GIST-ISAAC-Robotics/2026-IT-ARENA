# 인계: 최소 경기 운영(C)

작성 2026-11-02 06:40 UTC. 5시간 사용량 경고(0.90, 주간 0.69)에 따른 Claude→GPT 인계다. 결과 기준은 [README](README.md)다.

## 실행 상태

- **커밋·푸시·외부 게시·Jetson·전원 변경 없음.** HEAD `59c3c3b0`, 미커밋 작업 트리.
- 소유 프로세스 없음. `gz sim`·`ros2 launch`·`race_ops`·대역 차량 잔존 0을 확인했다. Haiku 보조 실행은 모두 끝났다.
- **완료:** 대표 흐름 a6 평가 37개 통과, 관련 시험 238 통과, 소스 계약 연결.
- **미완료:** 아래 표. C를 끝났다고 표시하지 말 것. 시뮬 필수 동작은 연결됐고 현장 항목과 마커 건너뜀 분기가 남았다.

## 변경 파일

| 구분 | 파일 |
|---|---|
| 제어 수정 | `src/arena_autonomy/arena_autonomy/`: `race_session.py`, `local_pursuit.py`, `wall_follow.py`, `vision_process.py`, `lidar_motion_ros.py`, `lidar_safety.py`, `local_follow.py`; `setup.py`; `src/arena_bringup/launch/g1_follow.launch.py` |
| 새 파일 | `race_ops_core.py`, `race_ops_node.py`, `race_ops_page.py`, `race_ops_export.py`; `scripts/race_ops_eval.py`, `scripts/race_event_export.py`; `tests/test_race_recovery.py`, `test_race_ops.py`, `test_race_ops_node.py`, `test_standing_wall_reacquire.py` |
| 수정 | `scripts/race_operator.py`, `tests/test_local_follow.py`(대역에 `prepared_output`), `tests/test_source_provenance_r0_plus_b3.py`, `.gitignore`(이 폴더 원시 기록·토큰 제외) |
| 문서 | `docs/PROJECT_CONTEXT.md`, `docs/DEVELOPMENT_PLAN.md`, `docs/autonomy/RACE_MONITOR_AND_RECOVERY_PLAN_2026_11_02.md`, `docs/autonomy/RACE_OPERATOR_RUNBOOK.md`, `docs/activity/2026-11-02.md` |

## 검증 상태

| 항목 | 상태 |
|---|---|
| 최초 신호 없는 START 거부 | 완료(a6) |
| 같은 경기 STOP/RESET/최종 배치/START 1회, 왼쪽 ID30 운반 포함 | 완료(a6, 평가 정답) |
| ID30 안 보이는 오른쪽 재개 | 완료(a6, 평가 정답) |
| 오른쪽 프로세스 재시작 후 복구 | 완료(a6, 평가 정답) |
| 준비 중 새 STOP·연결 상실·응답 유실·중복 STOP | 완료(a6) |
| 경계 뒤 입력만 사용 | 완료(판단 275건, 영상은 세대·시각 시험) |
| 화면 버튼 | 대역 차량 Chrome 완료. a6 후 소규모 표시 수정은 브라우저 재확인 안 함 |
| 실제 브라우저로 Gazebo 주행 조작 | **미실행** |
| 사건 내보내기 | a6 CSV/HTML 생성. 누락 표시는 벽시계 기준이라 저RTF에서 과다 |
| 수정 뒤 전체 회귀 | **미실행.** 수정 전 16 실패(기존 13, 새 3). 새 3은 집중 시험에서 해소 |
| 공용 AP·보안·실제 MCU·실제 정지·Jetson | **미검증** |
| 교통 후보 G1/G2와 재개 결합 | **미실행** |

실패 기록 보존: a1 `services` 이름 충돌, a2 ARM 뒤 DISARM, a3 대역 시험 `pkill`로 무효, a4 평가기 인자 충돌, a5 합류부 영구 정체. a5 원시 기록은 `tests/test_standing_wall_reacquire.py`의 재생 입력이다.

## 현재 고장·가설

- 알려진 진행 차단 결함은 없다.
- **미해결 분기:** 왼쪽 상태로 멈춘 차를 ID30 표지판 너머로 옮기고 마커를 보지 못하면 `side`가 왼쪽이다. 합류부 잘못된 벽 추종 가능성은 **추측이며 미실행**이다. 첫 단계는 아래 진단 장면이다. 좌표·세이브포인트 번호·운영자 좌우 지정으로 풀지 않는다.
- 어댑터 SIGINT 종료 지연(exit -9): 원인 미조사. HTTP `server.shutdown()`이나 `ros2 run` 신호 전달을 의심하며 추측이다.
- 재획득 옵션은 ON일 때 대표 조건만 봤다. 기본값 활성화 판단은 하지 않았다.

## 재개

WSL에서 저장소 루트 기준이다. 다른 시뮬과 동시에 돌리지 않는다. RTF 약 0.09라 대표 흐름이 약 8분 걸린다.

```bash
A=artifacts/validation/2026-11-02/official_ops_v1
bash "$A/build.sh"                                  # colcon + 관련 시험
bash "$A/final_checks.sh"                           # 관련 238건
bash "$A/run_eval.sh" d1 --scenario across_marker   # 마커 건너뜀 진단(미실행)
bash "$A/run_eval.sh" a7                            # 대표 흐름 재실행
python3 -I -B "$A/goal_eval.py" "$A/a7"             # 사후 평가
```

실제 화면:

```bash
ros2 run arena_autonomy race_ops --ros-args -p vehicle_id:=car1 -p host:=0.0.0.0 -p port:=8765 \
  -p token_path:=/path/token.txt -p record_path:=/path/race_record.json -p log_path:=/path/ops_log.jsonl
```

브라우저 재확인은 `stub_check.sh`를 띄운 뒤 Windows에서 `node page_check.mjs http://localhost:18770/ stubcar stub-token-12345 stub_steps.json stub_page`다.

제안 순서: 전체 회귀 1회 → GPT 검토 → `across_marker` 진단과 판단 → 교통 후보 결합 또는 다른 미완성 필수 기능. 단독 10 km/h·42–44 m 진단 재개점은 계획에 그대로 있다.

## 소스 계약

사슬 끝은 `config/tests/official_ops_r1_sha256.json`이고 부모는 `official_traffic_eval_r2_sha256.json`이다. 변경 10파일의 편집 전 바이트는 `before.zip`, 새 의존성 4개는 해시로 남겼다. `local_follow.py`와 `g1_follow.launch.py`의 편집 전 바이트는 백업 전에 고친 부분을 되돌려 만든 뒤 사슬 시험으로 대조했다. 소스를 다시 바꾸면 새 후속 계약을 추가하고 `M3_SUCCESSORS`에 잇는다. 이 계약을 고쳐 쓰지 않는다. a6 이후 소스 바이트 변경은 없다(시험 파일만 변경).
