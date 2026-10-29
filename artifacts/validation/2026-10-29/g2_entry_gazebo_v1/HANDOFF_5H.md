# 5시간 사용량 경고 인계(2026-10-29)

Claude Opus 5.5 구현 워커가 `rateLimitType=five_hour, utilization=.91` 경고로 새 작업을 멈추고 GPT에 넘긴다. 커밋·푸시·외부 게시·Jetson/전원 변경은 없었다. 기존 dirty tree·원본·실패 기록은 건드리지 않았고 reset/clean도 하지 않았다.

## 끝난 것

1. **신호 README 단정 정정:** [signal_spec_1029_v1/README.md](../signal_spec_1029_v1/README.md) "판단과 남은 범위" 첫 항목. 뒤쪽30 cm 카메라·도로 중앙 XY 가정에서 ID45 보존/제거 대조로 원인을 분리했다는 표현으로 바꿨고, 앞쪽 0.145 m 카메라의 수직 화각 부족은 별도로 남는다고 적었다. `docs/PROJECT_CONTEXT.md` 64행에는 원래 "시설 배치 문제" 단정이 없어 고치지 않았다.
2. **감속 원인 표현 정정:** 이 폴더 README의 "관찰한 동역학과 한계" 두 번째 항목. 노면 경계를 넘는 시점과 함께 관찰된 것이며 접촉력별 단독 원인은 확정하지 않았다고 고쳤다. 노면 고도·물성 변경은 후속 메모로만 남겼다.
3. **평가 v2와 음성 반례:** [g2_entry_left_v1/](../g2_entry_left_v1/README.md)의 `g2_eval_core.py`(주행 창/STOP 창 분리, 0.05 s 연속성, `/drive`·`/drive/safe`·판단 출력 상한, 헤더 설정 일치, 하네스·고정 표적 유효성, 우회 방향 부호), `g2_eval_v2.py`, `test_g2_eval_core.py`. WSL unittest 15/15 통과(`core_tests.log`, exit0). 첫 시도는 CLI 인자 순서 오류였으며 `core_tests_attempt1_cli_error.*`로 보존했다.
4. **run1 사후 감사:** `g2_entry_left_v1/run1_followup_audit_v2.json` 결과는 통과(18판정+무결성)다. 주행 창 안 중앙 유지는 3.840 s이고, 1 s 유지를 충족한 시각은 초록 후 9.162 s로 12 s보다 앞선다. STOP 창은 0.966 s이며 진행은 창 안 3.900 m, STOP까지 3.985 m다. 원래 `run1_g2_evaluation.json`·`run1/report.json`·`run1_sources.zip`은 SHA-256이 그대로다(`run1_audit_after_check.log`). 사후 기준이므로 run1의 등록 기준으로 쓰지 않는다.
5. **왼쪽 +0.149 m 등록:** `g2_config_v2_left.json`, `prepare_fixture.py`, `preflight.py`(통과: 표적 +0.149 m, launch 사본이 run1과 바이트 동일, `src/` 243파일이 run1과 동일하고 추가 0, 기타 저장소 소스 변화 0), `run.py`, `run1_prepared.json`(1082파일 스냅숏). 등록 내용은 왼쪽 README의 "왼쪽 run1 등록" 절에 있다.

## 진행 중이던 실행(인계 시점)

- 왼쪽 run1은 Claude Haiku 5.5 보조가 `run1_trial.sh`로 1회 실행했다. 인계 시점 WSL PID는 `bash run1_trial.sh` 606 → `run.py` 738 → `ros2 launch` 756 → `gz sim` 797이었다(약 1분 35초 경과). 정상적으로 끝나면 스크립트가 `run1_exit.txt`·`run1_residual_processes.txt`·`run1_g2_evaluation.json`·`run1_path.png`·`run1/trial_overlay.json`을 남긴다.
- 결과는 아래 "왼쪽 run1 회수"에 적는다. 그 절이 비어 있으면 실행이 끝나지 않은 것이다. 이 경우 같은 스크립트를 다시 돌리지 말고 먼저 `run1_exit.txt`의 존재와 위 PID의 잔존 여부를 확인한다. 스크립트는 `run1/`이 이미 있으면 exit 90으로 멈추므로 재실행해도 덮어쓰지는 않는다.

## 왼쪽 run1 회수

**실패(G2 v2 등록 판정).** 1회 실행했고 재실행하지 않았다. `run1_exit.txt`=1이며, 실행 전과 후의 잔존 프로세스 기록은 모두 비어 있다. 하네스는 유효했고 runtime_contract와 소스 불변도 true다. 실행 무결성·STOP·종료 검사에서 실패한 항목은 없다. 근거 파일은 [평가](../g2_entry_left_v1/run1_g2_evaluation.json), [그림](../g2_entry_left_v1/run1_path.png), [로그](../g2_entry_left_v1/run1_trial.log), `g2_entry_left_v1/run1/` 원본이다.

- **통과한 것:** 오른쪽 공간으로 우회해 초록 후 4.52 s에 표적을 지났고(통과 시 횡위치 −0.165 m, 부호 정상), 최대 횡편이는 −0.165 m(4.18 s)였다. 최소 차체 간격 159.9 mm, 접촉·겹침 0, 세 접지면 해석 모두 도로 밖 바퀴 ≤2, 벽 바퀴 0이었다. 명령 상한(`/drive`·`/drive/safe`·판단 출력), 헤더 일치, schema4 재생 동일, 고정 표적 유효도 확인했다.
- **실패한 것:** `returned_to_center_in_window`와 `stop_window_within_center_tolerance`. 중앙 조건(50 mm/0.10 rad)을 한 번도 만족하지 않았고(연속 구간 0), 창 끝 횡위치는 −0.1646 m, STOP 끝은 −0.1646 m/−15 mrad였다. 주행 창 진행은 **1.750 m**로 오른쪽 run1의 3.900 m보다 짧고, STOP 창 진행은 0이다. 통과 직후 차폭만큼 비켜 선 자리에서 멈춘 것으로 보인다. 상태 단계는 `entry→rejoin`까지 갔지만 `centered`에는 이르지 못했다. `entry_guide_input_reset`은 상태 행 3회로 오른쪽 run1의 1회보다 많다.
- **원인 분석은 하지 않았다(인계).** 우선 볼 곳은 다음과 같다. ① `run1/g2_decisions.jsonl`에서 통과 이후 판단 reason과 speed 0 시점, reset 시점과 측정 속도. ② `run1/trace.json`의 `safety_reason`(독립 보호층 정지인지 제어기 정지인지). ③ 오른쪽 벽까지의 거리와 rejoin 후보 거절 사유. 좌우 비대칭(외벽 재정합이 한쪽 벽만 쓰는지 등)이 의심되지만 **추측**이다. 같은 조건을 반복하지 말고 원본 분석을 먼저 할 것.
- 원래 G1 `stopped_center` 판정도 실패로 보존한다(hold·inside_road·ego_stays_behind false, approached true).

## 미완료(GPT 인계)

- 왼쪽 결과를 해석하고 기술 보고서에 반영하는 일: 왼쪽 README 결과 절, `docs/PROJECT_CONTEXT.md`의 G2 문구(40·77–79행), `docs/DEVELOPMENT_PLAN.md`의 10/29 우선순위 G2 항목, 10/29 활동 기록. 아직 하나도 갱신하지 않았다.
- run1 README 표의 "4.81 s 유지/진행3.98 m"에 STOP 포함 주석과 사후 감사 링크를 다는 일. 원래 수치는 그대로 두고 주석만 덧붙인다.
- 비사각형 대표 표적은 설계 조사만 하고 구현하지 않았다. 조사로 확인한 사실은 다음과 같다. 하네스 `scripts/validate_g1_traffic.py`의 `actor_sdf()`는 단일 box collision 하나와 contact sensor `<collision>collision</collision>`로 구성되고, 196·220·876·884·982행에서 actor footprint를 `rectangle(length,width)`로 계산한다. 실행 LiDAR는 `lidar_link` pose (0.06, 0, 0.095), 500 samples/360°다. 자차 차체 visual은 0.188×0.13×0.06 m로, 차체 중심 z=0.0625이고 바퀴 반경은 0.0325 m다.
  - 후보 설계는 세 가지다. ① 작은 중앙 몸체와 바깥으로 돌출한 바퀴. 바퀴 상단 6.5 cm < 스캔면 9.5 cm이므로 **돌출 바퀴는 LiDAR에 보이지 않는다.** 실행 전에 기록해야 할 감지불가 반례다. ② 낮은 바닥판과 기둥 4개, 그리고 스캔면 위 상판. 스캔면에는 기둥만 여러 반환으로 보인다. ③ 비스듬히 정지한 차량(yaw와 조향된 앞바퀴의 회전 footprint).
  - 모든 후보에서 평가 진실은 높이와 무관한 전체 2D footprint 합집합으로 두고, 가시 외형은 z=0.095 m 단면으로 따로 둔다. 다부품 contact sensor가 `<collision>`을 여러 개 받는지는 기존 `scripts/probe_static_actor_contact.py` 방식으로 확인해야 한다(미확인).
- 중앙 정지 표적 정체, 작은 음수 측정 속도에서 G2 기억이 초기화되는 문제, 공식 잔디6 mm 노면 반영 여부는 그대로 미해결이다.

## 주의

- WSL 실행은 `wsl.exe -d Ubuntu-24.04 -- bash -lc "bash '<스크립트 /mnt 경로>'"`로 한다. 인라인 `$PWD` 인용은 실패한 적이 있어(`run1_prepare_attempt1_quoting.log`) 스크립트 파일로 실행한다.
- `g2_eval_v2.py`는 `g2_entry_gazebo_v1/g2_eval.py`의 기하 보조 함수를 불러 쓴다. 그 파일은 수정하지 말 것(스냅숏에 포함됨).
- 왼쪽 실패 시에도 같은 조건을 자동으로 반복하지 않는다.
