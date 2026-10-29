"""closed1 합성 폐루프 결과를 정책별로 대조한다. 새 주행 아님.

기존 정책(front_patch_v0)의 기본 4조건은 g2_handoff_v1/prototype2와 같은 궤적이어야
한다(integration3에서 연결 코드=시제품 판단을 확인). 이는 비교 장치의 재현 확인이다.
"""
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
PROTOTYPE = ROOT/'artifacts/validation/2026-10-29/g2_handoff_v1/prototype2/report.json'


def main():
    closed = json.loads((BASE/'closed1/report.json').read_text(encoding='utf-8'))
    old = {c['name']: c for c in json.loads(PROTOTYPE.read_text(encoding='utf-8'))['cases']}
    rows = []
    for case in closed['cases']:
        entry = case['entry'] or {}
        row = dict(case=case['case'], policy=case['policy'], progress_m=case['final_progress_m'],
            contact_samples=case['contact_samples'], max_grass_wheels=case['max_wheels_touching_grass'],
            min_gap_m=case['min_gap_m'], stopped=case['stopped'], first_prepared_s=case['first_prepared_s'],
            prepared_calls=case['prepared_calls'], first_ready_s=case['first_ready_s'],
            entry_s=entry.get('time_s'), entry_center_x_m=(entry.get('center_pose') or [None])[0],
            entry_target_gap_m=entry.get('target_gap_m'), entry_measured_speed_mps=entry.get('measured_speed_mps'),
            returned_to_follow=case['returned_to_follow'], last_follow_s=case['last_follow_s'],
            max_actual_speed_mps=case['max_actual_speed_mps'], transitions=[(t['time_s'], t['state']) for t in case['mode_transitions']],
            preparation_reasons=case['preparation_reasons'], schema5_replay=case.get('schema5_replay'),
            median_processing_ms=case['median_processing_ms'], max_processing_ms=case['max_processing_ms'])
        if case['policy'] == 'front_patch_v0' and case['case'] in old:
            ref = old[case['case']]
            row['prototype2_same_progress_contact_transitions'] = (
                ref['final_progress_m'] == case['final_progress_m'] and ref['contact_samples'] == case['contact_samples']
                and ref['max_wheels_touching_grass'] == case['max_wheels_touching_grass']
                and [(t['time_s'], t['mode']) for t in ref['mode_transitions']] == row['transitions'])
        rows.append(row)
    result = dict(closed_report_sha256=hashlib.sha256((BASE/'closed1/report.json').read_bytes()).hexdigest(),
        prototype2_report_sha256=hashlib.sha256(PROTOTYPE.read_bytes()).hexdigest(),
        source_unchanged=closed['source_unchanged'], rows=rows,
        scope='Synthetic sequential sensors, ideal bicycle and independent guard function. Not ROS/Gazebo/vehicle validation.')
    (BASE/'closed_comparison.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    for row in rows:
        print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in row.items()
                          if k not in ('transitions', 'preparation_reasons', 'schema5_replay')}))
        print('   ', row['transitions'], row['preparation_reasons'], (row['schema5_replay'] or {}).get('passed'))


if __name__ == '__main__':
    main()
