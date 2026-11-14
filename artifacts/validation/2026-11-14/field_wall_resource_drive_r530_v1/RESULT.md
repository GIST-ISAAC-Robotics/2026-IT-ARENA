# r530 구불벽·틈 교통 12초 결과

**원 판정은 실패입니다.** GREEN7.401→19.401의 정확한12sim초 동안2.88m 전진했으나, 최종 앞차 간격3.4217m로 `final_gap`만 충족하지 못했습니다. `harness_valid=true`, `run_valid=true`, abort/error 없음입니다. 초기 정지 상태에서 양수 주행으로 전환했지만 후반에는 독립 보호층의0.1m/s 제한이 이어졌습니다. 같은 조건을 재실행하지 않았습니다.

actor1·시설3개·첫 직선 두 벽만 변형한 고정 Gazebo 조건입니다. 진폭30mm·주기1m·두께5mm·600mm마다20mm 틈은 미실측 가정 민감도입니다. 전체 교통 랩·추월 완료·전 코스 벽 변형·실차 증명으로 확대하지 않습니다.

| 항목 | 원 기록 |
| --- | --- |
| 진행거리 | 2.88m, projection jump0 |
| 최초 ground motion | 7.901sim초 |
| 원 평가 최장 정지 | 0.100sim초 |
| 최고 저장 ground speed | 1.091017m/s |
| 최소 표본 간격 / swept 하계 | 0.2964997m / 0.2964987m |
| 최종 간격 | 3.4217349m, 원 `final_gap=false` |
| 나머지 원 gate | 10개 true, 접촉/겹침/노면/진행/지속 포함 |
| 별도 추월 목표 | alongside/actor 통과/복귀 후 진행 미성립, 실패 |
| 명령 만료 | 0표본 |
| STOP / 정상 종료 | 원6개 모두 통과 /14개 child 모두 정상 |
| 강제 회수 / 잔여 소유 PID | 없음 /0 |

기존 별도 추월 평가기의 최장정지0.406초는 원 평가0.100초와 정의·범위가 달라 대체하지 않습니다. 마지막 `FOLLOW` 상태만으로 추월 완료나 지속적인 정상 속도 회복을 판정하지 않습니다.

## 제어 발행과 독립 보호 출력

[보충 집계](controller_guard_supplement.json)는 저장 자료 분석이며 제어 재생은 하지 않았습니다. 계획127개 중 helper1개, helper 없는 양수 정상 경로125개,0출력1개입니다. 정상 경로는 `handoff_state=FOLLOW`, `normal.reason=local_follow`, helper 없음, 양수 속도로 구분했습니다.

실제 route publication251개 모두 양수이고, 모든 발행은 직전 call의 command reference와 일치합니다. helper 발행은 t7.73/t7.82의0.1m/s 두 개입니다. t7.922부터 helper 없는 정상 경로가 발행됐습니다(actual0.2m/s, checked0.45m/s). 정상 경로 발행은249개입니다.

최초 `known_obstacle_latched`는 t7.52이며 당시 requested speed0, 사유는 `zero_command_preserved`였습니다. 독립 보호 양수 출력은 t7.74, `normal_cap_restored=true` 최초 기록은 t9.52입니다. 최초 래치가 발생하지 않았다고 주장하지 않습니다.

요청이0.1보다 크지만 guard가0.1로 제한한 연속 저장 구간은 t7.94–9.50,11.24–13.10,13.86–13.90, **14.02–19.38**입니다. 마지막 표본 시각 폭5.36sim초 동안 requested1.0m/s·미해결 기억점3개·`retained_memory_current_sweep_crawl`이 남았습니다. 표본 구간 폭을 실제 정지시간 합계로 해석하지 않습니다. 양수 요청을 guard0으로 만든 memory-command 원문 행과 양수 `/drive`/0 `/drive/safe` trajectory 행은 각각0개입니다. 실제 점유 위험에 대한 제동 효능을 이번 시험으로 입증한 것은 아닙니다.

[원 수집](evidence_collection.json)의 `positive_normal_follow_plans=0`은 `normal.selector_mode=='FOLLOW'`를 센 값입니다. 이번 selector는 `GUARD`이며 handoff `FOLLOW`와 다릅니다. 동결 수집기/원값을 보존하고 위 보충 집계의 schema 구분을 사용합니다.

원문: [최초 래치](guard_event_extract/first_latch.json), [최초 양수 보호](guard_event_extract/first_positive_guard.json), [최초 정상 cap](guard_event_extract/first_memory_normal_cap_restored.json), [마지막 제한](guard_event_extract/last_requested_above_crawl_cap.json). 전체 `run1/safety_status.jsonl`은1,015행/45,190,271바이트입니다. current/retained memory payload와 원래 시각을 필터링하지 않았고 추가 `receipt_monotonic_s`는 평가기 수신 시각입니다. guard 내부 callback 순서는 아닙니다.

## 입력·자원·회수

월드는 r520과 기하가 같고 마커 PNG 경로 네 곳만 수정했습니다. 준비본 SHA `c32c19343d488a0e3d4909c733b1d1f37efcf2ea2822ccd7dff33c934a299f09`, runtime475벽 collision digest `1f1c4b99085e29f89ed335202240de89c3360fdc464cda222075fb079348adc4`가 일치합니다. [실제 자원 참조11건](runtime_resource_receipt.json) 모두 존재/SHA 검사를 통과했고 종료 감사 error는 전후0개입니다. r520 원 오류·실패는 보존했습니다.

Root가 확인한 공통457회귀와 r531 저장 근거를 포함하여 입력1,484개를 동결했습니다. r523/r524/r526/r529/r532 제어·r535 비제어 UI/DOM·명시 fixture 보완을 선택했고 예상 밖 기존 입력 차이는0개였습니다. 실제 import/경쟁0을 확인했으며 native 재빌드는 없었습니다. 실제 snapshot266개 입력은 ZIP entry와 SHA가 모두 일치하며 파생 월드·자산·로컬 계측 runner를 포함합니다.

계획 제출/완료127/127, 누락·중복0, call 입출력 SHA 불일치0입니다. footer accepted/written1127/1127, dropped/failed/pending/capture_errors0, worker_alive=false입니다. 기록 완전성을 원 주행/STOP/종료 판정과 구분합니다.

PID18825/partition `arena_autonomy_test_18825`. UTC04:01:45 기동,04:06:13 wrapper exit1입니다. report의 host wall elapsed248.908초와12sim초를 구분합니다. 조기 STOP helper 없이 원 평가기의 STOP을 사용했습니다.14/14 child 정상 종료·강제회수 없음·소유 PID0·동결1,484개 변경0/ZIP동일을 확인했습니다. 수집 wrapper exit0 뒤 WSL을 root에 반환했고 추가 주행은 없습니다.

| 파일 | SHA256 |
| --- | --- |
| prepared.json | `16242c2dac8b499d8ab20fd5d24b48bf150d27937c93c215325657d9bea49d90` |
| sources.zip | `bedd2f828f32e7ac564f976d86aa4612097eed74878f888a719dcbce5766ab9f` |
| run1/report.json | `a16b3c60df15bd137a45e94cb6523fc8aed252a4d160634459999c43db124229` |
| run1/source_snapshot.zip | `2dd1917ebd21c83958970d7d4b1e0cba055627eaf5222ccc1218963c99ac9edc` |
| run1/safety_status.jsonl | `bf6f1df27427b9a938eb3f58d0fe925123e7e1a4704c57dfbc2889efdc91b6c5` |

근거: [원 보고서](run1/report.json), [별도 목표](assessment_summary.json), [무결성](integrity.json), [PID 회수](execution_v1/owned_processes_after.json), [각 수집 코드](execution_v1/collection_exit.json), [root 인계](root_execution_review.json).
