from pathlib import Path

p=Path('docs/PROJECT_CONTEXT.md');s=p.read_text(encoding='utf8')
old='[후반 경계 전환 감사](../artifacts/validation/2026-11-08/corner_r46_transition_audit_v1/README.md)에서 37.62–38.50초 현재 관측과 기억 만료를 분리하고 있다.'
new='[후반 경계 감사](../artifacts/validation/2026-11-08/corner_r46_transition_audit_v1/README.md)는 현재 왼쪽 0.651 m 코너가 x 비교 때문에 탈락한 뒤 기억이 만료되는 것을 확인했다. 조기 벽 전환 사본은 악화되어 미채택했고, 현재 벽 우선 후 대안을 비교하는 [r47](../artifacts/validation/2026-11-08/current_corner_fallback_r47_v1/PLAN.md)을 연결했다. 365검사·사본 1,635명령 일치·기존 빠른 548/중속 753명령 불변 뒤 같은32초1회를 실행 중이다.'
assert old in s;s=s.replace(old,new);p.write_text(s,encoding='utf8')
p=Path('docs/track/V2026_11_06_MIGRATION_2026_11_07.md');s=p.read_text(encoding='utf8')
old='[r46 저장 입력 감사](../../artifacts/validation/2026-11-08/corner_r46_transition_audit_v1/README.md)에서 37.62–38.50초 경계 상실의 길이·일치·기억 만료를 분리한다.'
new='[r46 저장 입력 감사](../../artifacts/validation/2026-11-08/corner_r46_transition_audit_v1/README.md)는 충분한 현재 왼쪽 경로가 기존 x 비교에서 탈락하는 것을 확인했다. 조기 전환 사본은 미채택했고, 현재 벽 갱신을 먼저 시도하는 [r47](../../artifacts/validation/2026-11-08/current_corner_fallback_r47_v1/PLAN.md)은365검사·사본1,635명령일치·기존빠른548/중속753불변 뒤 같은32초1회실행 중이다. 이 실제결과를 먼저 검토한다.'
assert old in s;s=s.replace(old,new);p.write_text(s,encoding='utf8')
p=Path('artifacts/validation/2026-11-08/current_corner_fallback_probe_v2/README.md')
s=p.read_text(encoding='utf8').replace('작업 중.','선택형 연결 후 검증 중.')
s+='\n7표적검사·1,635판단 재생에서20명령이 바뀌고 최초37.52초였으며 경계미확정0명령45→30으로 줄었다. 이를 [r47](../current_corner_fallback_r47_v1/PLAN.md)에 연결해365관련검사·사본명령일치·기존빠른548/중속753명령불변을 확인했다. 실제주행 결과는 별도다.\n'
p.write_text(s,encoding='utf8')
print('r47 running state recorded without changing registered inputs.')
