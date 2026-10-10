# HANDOFF — 시뮬레이션 시계 격자 고정 요청 (scheduled_phase_v1)

마감 상태와 후속 작업 진입점. 기존 실행은 다시 시작하지 않는다.

## 시작 상태
- HEAD `25dea60`, 시작 `git status`/수정 파일 SHA: `start_git_status.txt`, `start_head.txt`, `start_tracked_modified_sha256.txt` (이 폴더). 과거 dirty 상태는 사용자 것이며 이번 변경과 구분한다.

## 사전 조사 결론 (실행 전)
- `lidar_safety`는 sim 시간 20 ms 타이머(`create_timer(.02)`)로만 `/drive/safe`를 낸다. 직전 실행들의 safe stamp는 모두 sim 20 ms 배수(위상 0)였다.
- 기존 평가기는 `now-last>=.019` 폴링 발행이라 요청 위상이 실행마다 떠다녔다. 시작 시각도 `ARM+1 s`로 격자 정렬이 없었다.
- MCU(`virtual_mcu`)는 `/actuation/tx` 수신 즉시 목표를 바꾸고 `/actuation/status`(now_us 포함)를 발행한다. 기존 분석의 MCU 지연은 20 ms 진단 표본 해상도였다.
- `guarded_sim_actuator`는 벽시계 10 ms 타이머로 `/sim/cmd_vel`을 낸다. 플러그인은 다음 PreUpdate에서 적용한다. 플러그인 적용 시각은 마지막 0목표만 `zero_steps`로 step 단위 추정 가능.

## 현재 단계
1. [완료] 스케줄러 `scripts/scheduled_command.py`, probe opt-in `--command-schedule sim_grid --schedule-phase-ms {0,10}`, 계약 `config/tests/scheduled_command_r0_sha256.json`, 회귀 `tests/test_scheduled_command.py`·`tests/test_scheduled_phase_analysis.py`, `test_physics_step_option.py` 계약 연결. `run_unit.sh unit_dev2.log` 172 통과.
2. [완료] 등록 README 동결(`pre_run_freeze.sha256`), pilot R1 `o1ms_p10_a` 통과(Sonnet 5.5 실행), 관문 추가문 동결(`addendum_pilot_freeze.sha256`).
3. [완료] R2~R6 배치 종료. 6회 모두 정지·정상 종료했으나 R4·R6은 스케줄 감사 실패다. r0 실행기가 그 실패를 최종 판정에 넣지 않아 배치가 멈추지 않은 결함을 보존했다. r1 실행기는 수정했으며 Gazebo 추가 실행은 없다.
4. [완료] 최초 분석·1176개 전체 회귀 뒤 Claude 사용량 한도에 도달했다. 사용자 지시에 따라 GPT-6 Astra가 직접 인계받아 최종 분석 검토·실패 전파 회귀·현황/보고서/활동 기록을 마감했다. 최종 관련 회귀 190개 통과, `analysis_v2.json`·`final_review_v2.json`·`review_source_v2.zip` 참조. 새 시뮬레이션이나 다른 구현 워커를 추가하지 않았다.

## 핵심 발견
- R1: safe=발행+10 ms(고정), MCU 적용=safe와 같은 sim 시각, 출력=MCU+5~6 ms, 플러그인 최종 0=출력+1 ms.
- 출력 5 ms 지연 원인: MCU가 같은 now_us에 tick 상태(이전 목표)를 먼저 내면 `AppliedGuard.update`의 `stamp <= last_stamp` 거부로 수신 상태(새 목표)가 버려지고 다음 5 ms tick 상태에서 반영된다(R1 2270 ms 사례). 코드 수정 안 함(보호 의미 변경).
- 평가기 노드 시계는 자체 `/clock` 구독값보다 최대 15 ms 앞섰다(구독 처리 순서 차이, 유실은 아님).
- R1/R5 요청·safe·MCU 전환 시각은 일치하지만 1 ms 물리 결과는 주요 지표 7개 모두 허용치를 넘었다. 출력 시점 1 ms 차이와 미관측 플러그인 적용/물리 변동의 기여는 미분리다. R4/R6 스케줄 실패를 포함한 0.25 ms 근접 결과를 안정성 증거로 채택하지 않는다.

## 다음 진입점
- 현재 작업은 마감. 수정 r1 실행기의 지연·실패 전파를 실환경에서 확인한 뒤 플러그인 중간 전환 계측을 설계한다. 기존 여섯 실행과 원본 보고서를 덮어쓰지 않는다.
- 기본 구동/1 ms 월드·G1 꺼짐·차량/보호 계약은 유지. 왼쪽 가림 재대조·오른쪽/G2/고속·Jetson·전원·커밋/푸시는 이번에 하지 않았다.
- 이번 실행 담당은 사용자 확인대로 Sonnet 5.5였다. 향후 단순 실행 기본 요청은 Haiku 5.5이지만 지원 확인 없이 Haiku 4.5 등으로 대체하지 않는다.
