# 제어기 내부 계측·1배속 처리 개선 검증 자료

상세 조건·판정·한계는 [A2 보고서의 9/27 절](../../../../docs/simulation/RECORD_REPLAY_2026_09_24.md#2026-09-27-제어기-내부-계측과-1배속-적체-개선)을 기준으로 한다. 이 파일은 산출물/재현 경로만 안내한다.

| 경로 | 역할 |
|---|---|
| `baseline_stages/`, `baseline_io_stages/` | 수정 전 단계 계측과 1배속 실패 재현 |
| `native_logs_v1/` | 출력 저장 위치만 변경한 가설 검증. 개선되지 않았으며 임시 코드 제거 |
| `logger_profile/failure.json` | 최초 단독 로거 계측의 import 실패 보존 |
| `logger_profile_v2/` | 수정한 단독 로거 cProfile/요약, 설치 라이브러리 해시 |
| `queued_logging_v1/` | 로그 분리만 적용한 중간 재생 |
| `final_1x_a/`, `final_1x_b/`, `final_half/` | 최종 소스의 1배속 2회·0.5배속 1회 |
| `comparison.json` | 전체 7회 재생의 같은 정의에 따른 비교 |
| `initial_comparison.json` | 중간 판단에 사용한 비교 보존 |
| `regression.log` | 전체 회귀 출력 |
| `recording_failure/` | 미완료 기록 거절 검사 |
| `closed_loop_12m/` | 계측 비활성·가상 MCU·12 m 폐루프 주행/정지/종료 확인 |

재생별 `report.json`은 입력 전달·시간·EOF·종료·실행 소스 해시를, `source_snapshot.zip`은 당시 소스를 보존한다. 최종 세 실행의 `delivery_audit.json`은 원본 헤더를 대조한 누락 구간이다. 이전 실패의 소스/결과를 최종 성공으로 바꾸지 않는다. 원시 bag와 상세 기록 수신 로그의 기존 Git 제외 정책은 유지한다.

## 재현

WSL 저장소에서 `source install/setup.bash` 후 실행한다. 출력 폴더 이름은 기존 경로와 겹치지 않게 선택한다. 처리량 재생 중 Gazebo·pytest를 동시에 실행하지 않는다.

```bash
python3 scripts/replay_sensor_bag.py \
  artifacts/validation/2026-09-24/record_replay/capture_v2/sensor_recording \
  --rate 1.0 --output artifacts/validation/2026-09-27/controller_performance/repeat_new

python3 scripts/summarize_controller_performance.py \
  artifacts/validation/2026-09-27/controller_performance/final_1x_a \
  artifacts/validation/2026-09-27/controller_performance/repeat_new \
  --output artifacts/validation/2026-09-27/controller_performance/comparison_new.json

python3 scripts/profile_ros_logging.py \
  --output artifacts/validation/2026-09-27/controller_performance/logger_new
```

12 m 폐루프는 보고서의 기존 `validate_basic_autonomy.py` 명령에서 `--record-inputs`를 빼고 출력 경로만 새 이름으로 지정한다. 재생 결과는 폐루프 주행 또는 실차 인증이 아니다.

## 폐루프 소스 보존 보충

`closed_loop_12m/source_snapshot.zip`은 당시 검사기의 목록에 따라 생성되어 `queued_log.py`와 `timing_probe.py`가 빠져 있다. 두 파일은 폐루프 전에 저장된 `final_1x_a/source_snapshot.zip`에 포함되며 최종 A/B/0.5배속 보고서의 해시와 현재 파일이 모두 일치한다.

| `src/arena_vehicle_interface/arena_vehicle_interface/` 아래 파일 | SHA-256 |
|---|---|
| `queued_log.py` | `03c96124f5f5e2f4bbee7210e9ff1b16458c62537ef4ac15d00944d6a9b11300` |
| `timing_probe.py` | `f44c5b7b8ce9d84e47ff5ec67603844356b15877dca7b5627f80505ef531a351` |

기존 폐루프 보고서/ZIP을 사후 수정하지 않았다. 이후 `validate_basic_autonomy.py`의 보존 목록에 두 파일을 포함했으므로 현재 검사기 해시는 당시 보고서와 다르지만, 제어·센서·보호·구동 소스와 설정은 그대로다. 목록 보완 뒤 관련 회귀 출력은 `regression_manifest.log`에 있다. 다음 재현에는 현재 검사기를 사용하거나 당시 검사기와 위 보충 소스를 함께 사용한다.
