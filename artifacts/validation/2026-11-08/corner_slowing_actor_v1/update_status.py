from pathlib import Path

p=Path('docs/PROJECT_CONTEXT.md');s=p.read_text(encoding='utf8')
a=s.index('**현재 주행 작업:**');b=s.index('\n\n**그 다음:**',a)
s=s[:a]+'''**현재 주행 작업:** [r44](../artifacts/validation/2026-11-08/faster_entry_consistency_r44_v1/README.md)는0.70m/s 상대에게 추월 대신 FOLLOW로7.33m·정지0을 확인했지만 빠른추월은 미완료다. 코너 가림의 [r45](../artifacts/validation/2026-11-08/corner_occlusion_connected_r45_v1/README.md)·[r46](../artifacts/validation/2026-11-08/retained_corner_agreement_r46_v1/README.md) 실패를 보존한다. 현재벽 갱신 실패 뒤 충분한 다른 현재코너를 비교하는 [r47](../artifacts/validation/2026-11-08/current_corner_fallback_r47_v1/README.md)은365검사·사본1,635명령일치·기존빠른548/중속753불변 뒤 같은32초실행에서 **진행9.85m·최장저속0.704초·코너병렬0초로 실패**했다. 접촉0·STOP/정상종료·930입력불변이며 원report의9.89m/0.604초는 다른집계다. 기존 자차가20–70°를 돈 연속표본 자체가0.302초여서 상대출발위치만 바꾸는 저장기하에서는0.5초 병렬을 만들 수 없었다. 같은장면의 미세정지 보완을 더 반복하지 않고, [코너 감속 상대](../artifacts/validation/2026-11-08/corner_slowing_actor_v1/PLAN.md)의 고정각본을 별도 준비한다. 제어기는r47 그대로이며 새각본의 종료시간은거리/속도로사전산출한다. 기존실패를 새배치의성공으로바꾸지 않는다.''' +s[b:]
p.write_text(s,encoding='utf8')
p=Path('docs/track/V2026_11_06_MIGRATION_2026_11_07.md');s=p.read_text(encoding='utf8')
a=s.index('**11/8 r46 이후:**');b=s.index('\n\n**지금 먼저 확인:**',a)
s=s[:a]+'''**11/8 r47 이후:** [현재벽 우선 코너 대안](../../artifacts/validation/2026-11-08/current_corner_fallback_r47_v1/README.md)은365검사·사본1,635명령일치·기존빠른548/중속753불변 뒤32초실행에서9.85m·최장저속0.704초·코너병렬0초로 실패했다. 접촉0·STOP/정상종료·930입력불변은 확인했다. 현재벽이 살아 있는데 먼저 전환하던v1사본은미채택이며 과거r45/r46실패도보존한다. [시간배치 감사](../../artifacts/validation/2026-11-08/corner_pair_timing_audit_v1/README.md)는기록자차의20–70°구간자체가0.302초여서단순상대위치변경으로0.5초병렬을입증할수없음을확인했다. 다음은 제어기를그대로두고 [코너에서감속하는옆차](../../artifacts/validation/2026-11-08/corner_slowing_actor_v1/PLAN.md)를사전기하/거리기반시간계산뒤1회확인한다. 기존32초실패의시간연장이나판정선완화로해석하지않는다. 실제병렬도미완료이며단독속도·빠른추월·운영·지름길·최신Jetson/실물과제를유지한다.''' +s[b:]
p.write_text(s,encoding='utf8')
print('r47 result and next scenario recorded; production code unchanged.')
