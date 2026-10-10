# G1-b 등속 seed 0 정체 원인 분석 (재개, 2026-10-07)

대상 실패: [`g1b_pilot_v3_slow_s0`](../g1b_pilot_v3_slow_s0/report.json) — sim 8.020초 `follow_boundary_unconfirmed` 뒤 끝까지 정체. 이 폴더는 보존 원시 자료의 **open-loop 재생**과 점 단위 분석, 회귀 시험용 발췌를 담는다. 실행 결과나 행동 판정을 바꾸지 않는다.

## 재생 조건과 한계

- 입력: 보존 `scans_received.jsonl`(SHA-256 `36890dc9…`, report와 일치)과 가상 MCU `actuation_trace.json`의 평균 바퀴 속도. 50 Hz 제어 시각에 다시 공급했다([`scripts/replay_g1b_follow.py`](../../../../scripts/replay_g1b_follow.py), [`run_replay_v1.sh`](run_replay_v1.sh)).
- IMU 자이로는 기록되지 않았다. **0 자이로**와 평가기 자세 yaw 변화율의 **분석용 근사** 두 조건으로 돌렸고 결과가 같았다. 평가 자세는 제어 입력에 넣지 않는다.
- 제어 출력이 기록된 센서 흐름을 바꾸지 않으므로 폐루프 재현이 아니다. 8.3초 이후는 실제 차량이 멈춘 기하에서 제어기가 무엇을 판단하는지만 보여 준다.
- 이전 소스는 HEAD 추적 파일 + [재개 시작 ZIP](../g1b_resume_start_v1/source_before.zip)으로 펼친다(`local_follow.py` `ed524797…` = v4 freeze). 작업 트리 복사본과 41개 Python 파일이 같음을 [`check_overlay.sh`](check_overlay.sh)로 확인했다.

## 확인한 기전

| 시각 | 기록/재생 결과 | 근거 |
|---|---|---|
| 7.82 (scan 7.702) | 기억 기반 지원 선택이 1.8 m 경계를 만들어 경로를 갱신 | 재생 행, 기록 상태 7.96 `path_source_s=7.702`와 일치 |
| 7.92 (scan 7.802) | 왼쪽 벽 바로 앞 팀 임시 신호등 기둥 `tl_post`(2.5 cm, 벽면에서 약 7.8 cm 안쪽)의 바깥 모서리 반환 1점이 기대 벽에서 0.077 m라 지원 허용치 0.08 m 안에 남았다. 방위 순서에서 이 점→다음 벽 점 간격 0.159 m가 연결 한도 0.117 m를 넘어 후보가 0.506 m로 끊겼다. 기억(나이 0.218초)으로 주행 | [`debug_support.py`](debug_support.py) 출력, 7.62~7.82초에는 같은 기둥 점이 0.084~0.086 m라 제외됨 |
| 8.02 (scan 7.902) | 기억 나이 0.318초 > TTL 0.30초로 만료. 비필터 왼쪽 후보 0.504 m < 0.65 m, 반대 벽 1.8 m는 기둥 점까지 이어진 짧은 선호 후보와 `path_agrees`가 거짓이라 대안도 거절 → `follow_boundary_unconfirmed` | 기록 상태 8.02와 같은 값(`boundary_candidate_ahead_m` 0.504), [`check_alternative.py`](check_alternative.py)(7.902·9.802 scan 모두 일치 거짓) |
| 8.3 이후 | 정지 위치에서 비필터 왼쪽 후보 0.42~0.47 m. 무기억 재획득은 비필터 선호 벽의 0.65 m만 요구하므로 반대 벽 1.8 m가 있어도 영구 교착 | 이전 소스 재생 401틱 미확인, 9.0초 reset 뒤에도 376틱 모두 미확인 |

음수 피드백 reset의 재발, 센서 무응답, 보호층·간격 정지는 이 전환의 원인이 아니었다(측정 +0.389 m/s, motion `ok`, 보호층 `clear`). 기둥 모서리가 허용치 안에 드는지는 광선 위상에 따라 달라지는 경계 사례이며, 실제 원인은 (1) 지원 허용치 근처 물체 가장자리 한 점이 벽 연결을 끊는 선택 방식과 (2) 기억이 없을 때 그 끊김을 우회할 실제 관측 경로가 없는 재획득 조건의 결합이다.

## 수정 전후 재생 비교

| 재생 | 이전 소스 `ed524797…` | 1차 수정 `48ba2f44…`(최종 `fddc808b…`도 같은 요약) |
|---|---|---|
| 3.5~16.0초, 0 자이로 | 8.02초 경계 상실, 401틱 미확인, 양수 명령 220틱 | 경계 상실 없음, 양수 명령 600틱(0명령은 시작 확인·release·signed 역방향 HOLD) |
| 같은 구간, 평가 자세 yaw 근사 | 위와 동일 | 위와 동일 |
| 8.5초 무기억 시작 + 9.0초 reset | 376틱 모두 미확인 | 두 번 모두 `bilateral_reference_support`로 2회 확인 뒤 재획득 |

요약·핵심 행·상세 파일 해시는 [`replay_summary.json`](replay_summary.json)에 있다. tick 단위 상세 행(`replay_*_*.json`)과 이전 소스 펼침본은 로컬 보존·Git 제외다. 같은 구간 `LocalFollow.command` 계산 시간은 [이전](command_timing_old.json)/[1차](command_timing_new.json)/[최종](command_timing_final.json) 최대 8.32/12.97/10.05 ms(20 ms 초과 0)였다. 이전 소스는 상실 뒤 조기 반환이 많아 같은 작업량 비교가 아니며 PC WSL 측정이다.

## 1차 수정판 실행의 일시 상실과 최종 수정

1차 수정판(`48ba2f44…`, 반대 벽 기준 재선택은 무기억일 때만)의 [Gazebo 등속 seed 0](../g1b_resume_v1_slow_s0/report.json)은 통과했지만 sim 8.72초에 경계를 한 번 잃었다. 같은 실행 자료 재생([`run_replay_v2.sh`](run_replay_v2.sh), [`debug_abeam.py`](debug_abeam.py))에서 scan 8.502의 기억 경로는 기둥 바로 앞(현재 x≈0.167 m)에서 시작했고, 기둥 점(x≈0.09~0.11 m)은 **기억 경로 시작점 뒤 범위 밖 관측**이라 1·2차 지원 선택 모두에서 남아 지그재그 연결 → `wall_shape_uncertain`이 됐다. 최종판(`fddc808b…`)은 같은 스캔의 긴 반대 벽 기준 재선택을 기억이 있을 때도 쓰되 기억 경로 일치를 추가로 요구한다. [`run_replay_v3.sh`](run_replay_v3.sh) 결과: 1차판 실행 자료에서 상실 없음, 최초 실패 자료 해소·정지 기하 재획득 유지, 통과했던 정지 앞차 v3 자료는 526틱 전부 이전 소스와 같은 명령·이유. [`check_alternative.py`](check_alternative.py)는 기존 대안 거절(`path_agrees` 거짓)을 확인한다.

## STOP 전 MCU 고정 집계(seed 1 등속)

[`stop_latch_timing.json`](stop_latch_timing.json)은 각 재개 실행의 STOP 요청 시각과 첫 MCU 고정 표본을 비교한다. 모든 고정 표본 사유는 명시 STOP(`software_stop`)이고, seed 1 등속만 요청과 같은 밀리초(15.72초 / 15,720,000 µs)여서 평가기 `now_us*1e-6 < stop_start`가 `15.719999999999999 < 15.72`로 참이 되었다. 후보 원인 기록이며 `report.json` 판정은 바꾸지 않는다. 이후 평가기만 v4(요청 직전 새 `/clock` 정수 ns ↔ MCU 정수 µs)로 고쳐 새로 실행한 결과는 [G1-b 보고서](../../../../docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md#평가판-v3-stop-요청-시각의-정수-비교)에 있다.

## 회귀 자료

[`failure_window.json`](failure_window.json)은 보존 실패에서 이동 구간 scan 8개(7.202~7.902초), 정지 구간 scan 3개(9.802~10.002초), MCU 평균 바퀴 속도 120개만, [`transient_window.json`](transient_window.json)은 1차판 실행의 scan 5개(8.202~8.602초)와 속도 85개만 발췌했다([`extract_failure_window.py`](extract_failure_window.py)). 출처 파일 SHA-256을 함께 기록한다. 탐색용 스크립트(`explore.py`, `dump_points.py`, `debug_support.py`, `prototype_cases.py`)는 판단 경위를 남기기 위한 것이며 시험 판정에 쓰지 않는다.
