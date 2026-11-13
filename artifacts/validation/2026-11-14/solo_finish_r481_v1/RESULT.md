# r481 결과 — 현재 경로 판단과 중복 측방 정지 분리

**v2 격리 패치를 인계한다. 공통 미통합이며 새 주행은 수행하지 않았다.** 최초 지속 정지의 계획/같은 입력 current 연결은 복구됐고 기존 양수→0 회귀는 없었다. 저장된 정지 이후 입력 전체의 양수화나 실제 완주를 입증한 결과는 아니다.

## 변경과 원 실패

원 r477은 최대41.96m 후 정체해 사용자 요청 STOP으로 회수된 미완주다. 최초 지속 정지 `(355,1293)`,51.22s에는 현재 경로36점·관측 길이1.898m·swept clearance .80m·목표 .80m/s가 있었으나, 추가 측방 관측 관문의 `lateral_metric_outside_trial`로0이 됐다. v2는 `_partial_command`에서 현재 경로/실제 점유/제동 결과를 먼저 선택한다. 기존 `requested_speed/evaluate`의 입력·순서·기억 갱신은 유지하고, 그 뒤 중복 lateral veto의 `_hold` 부작용에 들어가기 전에 선택 명령을 반환한다. 경로/실제 점유의0, 전방 반환 전체 소실, 입력 무결성과 STOP은 유지한다. 원 ray/free·미관측·측방 상태를 덮지 않는다.

변경은 `local_follow.py`, `race_runtime_policy/current_route_priority.py` 두 파일뿐이다. [changes.json](changes.json)의 before/after SHA와 [change.patch](change.patch)를 **검증된 v2 최종 이관본**으로 확정한다. stage 231개 중 나머지229개 및 기존 native binary는 원 r477 동결본과 동일하다.

## 이미 완료된 검증

| 항목 | 결과 |
|---|---|
| 원 baseline 순서 재현 | 계획540/540, 기록 current854/854 **전체 출력 exact** |
| v2 계획540 | 기존 양수→0 **0**, 0→양수4, 기존 양수 증가6/감소0 |
| v2 기록 current854 | 848 exact, 기존 양수→0 **0**, 0→양수0 |
| 출력 상태/기하 | 측방 상태·handoff·선택벽·swept clearance 변화0, 경로6개 변화 |
| 집중 검사 | **9 통과**: 실제 점유/전방 전체 소실/경로 소실/near-invalid/정책OFF·기억 갱신·STOP 포함 |
| 종료/무결성 | v2 exit0, 소유 잔류PID0, common231/stage/원trace/input·epoch 불변, WSL 반환 완료 |

최초 지속 정지51.22/51.32s는 계획 .80/.75m/s로 복구됐다. 같은 원 입력의 별도 current 진단도 각각 .80/.75의 `rebase_current_follow`였다. 이 진단에는 실제 후속 발행·차체 이동 기록이 없다. 전체 0→양수4 중 나머지2는 앞선16.72/16.82s다. 근거: [v2 summary](native_run_v2/candidate/summary.json), [같은 입력 current](native_run_v2/candidate/recovered_same_input_current.json), [종료 감사](native_run_v2/exit_audit.json).

## 경로6개와 남은0

경로6개는14.32~14.82s의 `static_side_current_lane` 대신 기존 현재 경로를 선택한 경우다. 모두36점·같은벽이며 최대 좌표 차이는 **.005154m**다. 목표속도는 .938775에서1.0~1.5m/s로 바뀌었다. 이는 선택 전환에 따른 실제 출력 차이이므로 exact라고 표현하지 않는다.

51.42~61.62s의 저장 입력에는0이103개 남았고 모두 `parallel_hold_baseline_unavailable ← follow_lateral_metric_outside_trial`다. 첫51.42s는 memory 경로여서 새 fresh 분기가 선택되지 않았고,51.52s는 경로가 없었다. 이후 같은 legacy 정지 사유가 지속됐다. 원 차량은 이미 정지한 상태이므로 후보가 실제 전진했을 때의 후속 관측과 같지 않다. **103개를 모두 양수로 맞추기 위한 추가 수정·재생은 하지 않았다.** 상세는 [기존 집계](v2_detail_analysis.json)에 보존했다. 다음 판단 자료는 원 출발 변형의 실제1회 주행이며 시작 여부는 주 담당이 결정한다.

## 보존한 실패와 한계

v1은 기억 갱신까지 생략해 기존 양수144개를0으로 만든 실패이며 미채택이다. `native_run_v1`과 `stage_v1_failed.zip`을 보존했고 성공으로 소급하지 않았다. Windows에서는 설치 NumPy의 기존2차원 `np.cross` 비호환으로7개 중단/STOP2 통과였으며, v2의9개 통과는 기존 Linux native 환경 결과다. 새 빌드·ROS/Gazebo·장치 접근·실물 검증·상위 문서 수정은 없었다. 이번 마무리는 저장된 집계만 읽고 문서화했으며 새 시험을 추가하지 않았다.
