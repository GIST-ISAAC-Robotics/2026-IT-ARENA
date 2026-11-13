# r465 actor0 단독1회 결과 — 9.54 m 미완주

**고정1회 주행은9.54 m에서 지속 정지하여 미완주했습니다. 원 report `passed=false`/run exit1을 보존했습니다.** 8.086 sim초 무진행을 관측하여 소유 STOP을1회 요청했고, STOP 안정·자식14개 정상 종료·잔여PID0·종료 시점1,348개 입력 불변을 확인했습니다. 이전 r457의23.35 m 구간까지 도달하지 못했습니다. 공통 소스·r457 원본·Jetson은 변경하지 않았으며 같은 주행을 반복하지 않았습니다.

## 실제 주행과 회수

- green 7.401 sim초, 첫 움직임8.133, 최대 진행9.54/44.64 m. 최대 명령1.5 m/s까지 정상 가속했으나 마지막 정지 뒤 재가속은 없었습니다. 원 평가의 최장 정지9.096 sim초, 누적 정지9.700 sim초입니다.
- 마지막 진행 증가19.155 sim초 이후27.241에서 무진행8.086초를 확인했습니다. 소유 STOP helper exit0, 평가기 STOP 기준28.391 sim초, 이후0.5초 정지 변위약3.22 μm 및 dense pose 근거 통과입니다. 벽 접촉0, two-wheel surface 판정 통과입니다.
- `/drive`와 `/drive/safe`가 관측됐으며 green 이후208개 trajectory 표본의 guard 이유는 모두 `clear`입니다. MCU 양수 명령 표본2,720개, reason은 `boot/explicit_arm/zero_target/drive/software_stop`뿐이고 `command_expired`는0입니다. 기한 진단 메시지0은 지연 부재의 전수 증명이 아닙니다.
- 소유 STOP이 평가기 자체 STOP보다 먼저 전달되어 원 `abort=mcu_latched_before_stop: software_stop`, 선행 latch2표본, `no_mcu_latch_before_stop=false`도 그대로 남겼습니다. 이를 성공으로 소급하거나 정체의 시작 원인으로 바꾸지 않았습니다.
- launch exit0/자식14개 모두 정상 회수, reader 강제 종료 없음, 소유 partition/PID0입니다. [종료 후 검사](execution_v1/check_after_run.json)는1,348개 입력 불변이며 [소유 프로세스 검사](execution_v1/owned_processes_after.json)도0입니다. WSL/공통 동결은 회수 완료 뒤 주 담당에게 반환했습니다.

## 최초 지속0 전후 연결

아래 reference/발행/MCU 시각은 simulation 기반 시간입니다. 계산 시간은 host monotonic 차이로, 다른 clock의 절대값을 서로 빼지 않았습니다. 전체 소규모 타임라인은 [timing_rows.json](timing_rows.json)에 있습니다.

| 근거 | 시각·출력 | 판단 |
|---|---|---|
| call821 / plan(361,292) | reference18.920, .35 m/s; 계산34.85 ms | RELEASED의 `released_observed_motion_small_masked_tail` |
| 같은 call의 current(361,293) | reference18.924, .35 m/s; 계산13.80 ms | `rebase_current_released` |
| route_publication822 | at18.940, 실제 .35 m/s | `published_checked_route` |
| submit823→complete824→call825 / plan(361,295) | reference19.020, 0 m/s; 계산31.71 ms | RELEASED, `selector_reason=large_motion_masked_toward_left`, 선택 parallel의 `parallel_hold_reference_not_straight`; current 호출 없음 |
| MCU 원 표본 |19.040, accepted_seq903, deadline19.140, target0 | `ARMED/zero_target`; 만료가 아닌 새0 명령 수신 |
| trajectory |19.055 drive0/safe0, safety clear | 마지막 양수 표본18.955의 .35에서0으로 전환; 정확한 부모0 발행시각은 별도 이벤트로 기록되지 않음 |

첫 거절의 region은 일반 미관측0/전면 mask0/왼쪽 mask1,089개/꼬리11.620 mm입니다. 원 baseline의 `target_speed_before_hold_mps`는약.5, swept clearance는.56 m였습니다. 원 selector가 1 cm mask 꼬리 관문에서 parallel 대안으로 전환하고 그 곡선이 직선 조건에 맞지 않아0을 반환한 것으로 로그가 연결됩니다. 이것을 물리 접촉 예측의 확정이나 helper 계산 폭주로 해석하지 않습니다.

reference19.02 이후94개 계획은 모두 RELEASED·같은 두 이유·0 출력이며 current 기록/양수 부모 발행은0입니다. 이94개 plan 계산 중앙29.34/p95 33.81/최대37.86 ms였습니다. 전체 기록의 current discard5개는 `retained_route_not_proven`3, `new_plan_superseded_current_check`2이며 모두 정체 전입니다(후자의 인접 call reference는13.32~13.42 및17.52~17.62). 기록된 late 폐기 이유는 없고 call delivery207개는 모두 candidate입니다. 기록되지 않은 사건의 부재까지 보증하지 않습니다.

## actor0의 측방 모드 전환

[mode_transitions.json](mode_transitions.json)에 원 selector 요약을 보존했습니다.

- call516/reference11.42: SOLO→GUARD. 오른쪽 유효 측방 반환3점, 경로 종방향 ds .3400~.3428 m로 `present=true`; 왼쪽2점은 false였습니다. call557/reference12.42에는 양쪽 `world_fixed_return=true`로 측방 seen 상태를 비우고 SOLO로 돌아왔습니다.
- call581/reference13.02: 양쪽 각4점(ds약-.32 m)으로 GUARD에 다시 들어갔습니다. call616/reference13.82에는 양쪽 present false, `vanish=rear`, passed true, clear좌2/우3을 근거로 RELEASED가 됐고 지속정지까지 유지됐습니다.

`pass_release_trial.threat_view`→`m3_execution_trial.ReturnGrid/side_return_masks`는 status `return`인 유한 점을 측방 창·compact·경로상 벽 제외 조건으로 셉니다. 선언된 `masked` 자체를 측방 물체로 센 경로는 아닙니다. 그러나 요약 로그에는 점 좌표/원 광선 번호/물체 ID가 없어 시설·경로 벽의 오분류·가림 밖 차체 중 정확한 대상은 미확정입니다. actor0와 `world_fixed_return`만으로 특정 시설의 검출이라고 단정하지 않습니다. 후속 RELEASED 설계는 주 담당이 맡습니다.

## 완전 기록과 비용의 범위

plan 제출207/완료207, 중복·미짝 식별자0, call이 참조한 원 입력/결과 대조 불일치0입니다. 제출·완료는 적용 증명이 아니며 실제 양수 route publication은111개입니다. footer는 accepted/written1,135, dropped/failed/pending/capture_errors0, writer 종료입니다.

전체 plan207개 계산 중앙29.06/p95 46.35/최대57.04 ms. 기록된 current111개는 중앙18.93/p95 32.15/최대130.16 ms(100 ms 초과1)입니다. r463 구조 수용은 출발 부근3건으로 current 전체 계산130.16/65.97/64.06 ms였습니다. 이 비용은 별도 개선 대상이지만, 이번 마지막94개0 계획에는 current가 없으므로 지속정지 원인과 구분합니다. 노드 전체 실시간·ARM·실물 비용으로 확대하지 않습니다.

[기계 집계](execution_summary.json), [원자료 SHA](raw_integrity.json), [읽기 전용 집계기](collect_result.py). 원 report SHA는 `1a84b760dc7d6b41da86ee69b58ddcdbc32d0ecbee02feae825b57cbfce06cb8`입니다. config/report의 설명 문자열에 초기 `integration pending` 문구가 남았지만, 실제 통합 source는 아래 승인 SHA와 archive로 동결되어 있으며 실행 뒤 문자열을 고치지 않았습니다.

## 실행 전 준비 근거

주 담당이 r463/r464 공통 통합과 r457 대비 실제 delta7을 검토한 뒤, 배정받은 WSL에서 고정 준비 wrapper를1회 수행했습니다.

r457의 runner/evaluator/finalize/시설/launch 등13개 파일은 byte exact입니다. config는 scope와 자기 entry 경로만, run은 자기 태그·baseline 기록만 바꿨습니다. actor0/시설3개·같은 최종 형상/C1/속도/물성/RTF·80sim초/8sim초 무진행STOP·기존 판정과 종료를 유지합니다. 새 방향·교통·속도 조건은 없습니다. 이전 WSL 첫 시작 실패·재시도·원 run1·PID/receipt를 새 실행으로 복사하지 않았습니다.

Windows `C:/Python314/python.exe -B .../verify_recipe.py`는 exit0입니다.13개 exact 사본, Python19파일 AST, bash LF, 같은 CLI/config와 시설3개를 확인했습니다. [recipe_manifest.json](recipe_manifest.json)은27개 recipe 파일 SHA와 고정 evaluator argv를 기록합니다. 이는 계산 입력의 최종 동결 manifest가 아닙니다. [원 비교](baseline_copy.json), [체크 로그](recipe_check.log), [고정 launch 명령](launch_command_expected.json).

2026-11-14 00:00 KST에 준비 세션이 exit0으로 끝났습니다. `arena_autonomy` 한 패키지 build0, 22개 모듈의 실제 import 경로/SHA와 native ABI/SHA 확인0, 같은 고정 CLI를 `rclpy.init` 직전에 차단하는 검사0입니다. ROS 초기화·Gazebo·양수 발행은 없었습니다. 공통1,309개 입력과 검토된 delta7을 대조했고, source archive 동결 후1,348개 입력 재검사도 통과했습니다. WSL 호출은1회이며 복구·재시도는 하지 않았습니다. 정책 회귀를 중복 실행하지 않았습니다.

준비 단계 근거는 [빌드/import 영수증](build_v1/receipt.json), [동결 재검사](build_v1/check_after_freeze.json), [Linux 회수](build_v1/session_cleanup.json), [최종 준비 영수증](build_v1/final_receipt.json)입니다. 준비 당시 Windows `wsl.exe` PID33924 exit0을 회수했으며, 이후 주 담당의 별도 실행 검토로 새 run을 등록했습니다. 주행 wrapper의 Windows PID38336은 원 판정 exit1로 회수했습니다.

| 동결 자료 | SHA-256 |
|---|---|
| `prepared.json` | `86dd064520e5f8f36ca99e02f4337a96e2710fe8e865d5df977368d57c54354d` |
| `sources.zip` | `60e2b3a277ceeb0ecb79dfa4bb54d3ad5f66b75ad86dc7d1ea376d28f52918f8` |
| `build_v1/receipt.json` | `cdd14bd43212f8eec52968c122a0475b8e40792d3dcdd14bdfa5cbffb081c47d` |

이번 고정1회 실행은 끝났습니다. 새 주행은 자동 반복하지 않으며 원 실패와 archive를 보존합니다. 후속 RELEASED 경로/가림 판단과 별도 비용 개선은 주 담당 검토로 넘깁니다.
