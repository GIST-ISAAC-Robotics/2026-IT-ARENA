"""정지 가장자리 성공 조건과 멈추는 앞차 실패 조건의 진입·모서리 통과를 대조한다.

closed1(g2_prepare_cap_v1)과 legacy_stop1의 schema5 기록·평가 기록만 읽는다. 표적 위치·
차체 간격은 평가용 정답이며 사후 분해에만 쓴다. 새 주행 아님.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
CLOSED = BASE.parent/'g2_prepare_cap_v1/closed1'
# 사례: (기록 위치, 정책, 표적 최종 중심 x). probe_closed_loop.CASES와 같은 정의다.
CASES = [('right_ideal', CLOSED, 'observed_entry_relevance_v1', 1.2),
         ('left_combined', CLOSED, 'observed_entry_relevance_v1', 1.2),
         ('right_combined', CLOSED, 'observed_entry_relevance_v1', 1.2),
         ('right_far_combined', CLOSED, 'observed_entry_relevance_v1', 2.4),
         ('left_far_combined', CLOSED, 'observed_entry_relevance_v1', 2.4),
         ('stopping_combined', CLOSED, 'observed_entry_relevance_v1', 1.7),
         ('stopping_combined', BASE/'legacy_stop1', 'front_patch_v0', 1.7)]


def analyse(name, folder, policy, target_x):
    stem = folder/(name+'__'+policy)
    records = {round(r['t'], 3): r for r in json.loads((Path(str(stem)+'.records.json')).read_text(encoding='utf-8'))}
    calls = []
    with open(str(stem)+'.jsonl', encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'call':
                continue
            data = event['data']; t = round(data['input']['reference_s'], 3)
            scan = data['input']['scan']; first = scan['sec']+scan['nanosec']*1e-9
            d = data['output'][2]; b = d.get('bypass') or {}
            cands = b.get('candidates') or []
            calls.append(dict(t=t, age=t-first, state=d.get('handoff_state'), reason=d.get('reason'),
                g2=b.get('reason'), selected=b.get('selected'), cands=cands, rec=records.get(t, {})))
    entry = next((c for c in calls if c['reason'] == 'handoff_entry'), None)
    if entry is None:
        return dict(case=name, policy=policy, entry=None)
    row = next(r for r in entry['cands'] if r.get('eligible') and r['offset_m'] == entry['selected']['offset_m']
               and r['transition_m'] == entry['selected']['transition_m'])
    eligible = sorted({(r['offset_m'], r['transition_m']) for r in entry['cands'] if r.get('eligible') and r['offset_m']})
    center = entry['rec']['evaluation']['center_pose']
    face = target_x-.1
    bypass = [c for c in calls if c['state'] in ('BYPASS', 'REACQUIRE') and c['t'] >= entry['t']]
    # 표적 옆 구간: 차체 중심이 표적 앞면-0.2 m부터 표적 뒷면까지.
    beside = [c for c in bypass if c['rec'] and face-.2 <= c['rec']['evaluation']['center_pose'][0] <= target_x+.1]
    holds = [c for c in beside if c['g2'] == 'no_entry_command_candidate']
    def pad(c):
        values = [r.get('maximum_occupancy_padding_m') for r in c['cands'] if r.get('maximum_occupancy_padding_m') is not None]
        return max(values) if values else None
    nearest = min(beside, key=lambda c: c['rec']['evaluation']['target_body_gap_m']) if beside else None
    return dict(case=name, policy=policy, entry_s=entry['t'], entry_selected=[row['offset_m'], row['transition_m']],
        entry_eligible_nonzero=eligible, entry_preview_progress_m=row['preview_return_progress_m'],
        entry_preview_point_gap_m=row.get('preview_point_gap_m'), entry_measured_speed_mps=entry['rec'].get('measured_speed_mps'),
        entry_rear_axle_to_target_face_m=face-(center[0]-.0675*1.), entry_target_body_gap_m=entry['rec']['evaluation']['target_body_gap_m'],
        beside_calls=len(beside), beside_g2_hold_calls=len(holds),
        beside_hold_scan_age_s=dict(Counter(round(c['age'], 2) for c in holds)),
        beside_pass_scan_age_s=dict(Counter(round(c['age'], 2) for c in beside if c['g2'] != 'no_entry_command_candidate')),
        nearest=None if nearest is None else dict(t=nearest['t'], target_body_gap_m=nearest['rec']['evaluation']['target_body_gap_m'],
            center_pose=nearest['rec']['evaluation']['center_pose'], scan_age_s=nearest['age'], max_padding_m=pad(nearest)),
        passed_target=any(c['rec'] and c['rec']['evaluation']['center_pose'][0] > target_x+.2 for c in bypass),
        final_center_pose=calls[-1]['rec'].get('evaluation', {}).get('center_pose'))


def main():
    results = [analyse(*case) for case in CASES]
    sources = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),)}
    (BASE/'corner_passage.json').write_text(json.dumps(dict(results=results, script_sha256=sources,
        scope='Post-hoc decomposition of stored synthetic closed-loop traces; evaluation truth used only for analysis.'),
        indent=2)+'\n', encoding='utf-8')
    for r in results:
        print(json.dumps(r, ensure_ascii=False))


if __name__ == '__main__':
    main()
