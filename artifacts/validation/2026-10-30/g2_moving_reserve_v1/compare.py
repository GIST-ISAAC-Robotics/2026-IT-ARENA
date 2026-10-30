"""보존된 비교 출력만 읽어 명령 차이와 폐루프 손익을 요약한다."""
from collections import Counter
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def shadow():
    old, new = [read(BASE/f'shadow_{v}/report.json') for v in ('existing', 'motion_v1')]
    counts = Counter(); changed = []
    files = [BASE/f'shadow_{v}/rows.jsonl' for v in ('existing', 'motion_v1')]
    rows = [[json.loads(line) for line in p.read_text(encoding='utf-8').splitlines()] for p in files]
    if len(rows[0]) != len(rows[1]) or old['input_sha256'] != new['input_sha256']:
        raise ValueError('unmatched input')
    for a,b in zip(*rows):
        if (a['seq'], a['reference_s'], a['expected']) != (b['seq'], b['reference_s'], b['expected']):
            raise ValueError('unmatched call')
        counts['calls'] += 1
        if a['requested'] != b['requested']:
            counts['changed'] += 1
            counts['zero_to_positive'] += a['requested'][0] == 0 < b['requested'][0]
            counts['positive_to_zero'] += b['requested'][0] == 0 < a['requested'][0]
            counts['steering_changed'] += a['requested'][1] != b['requested'][1]
            counts['candidate_exceeds_g1'] += b['requested'][0] > b['expected'][0]+1e-9
            counts['changed_without_motion_release'] += (b.get('reserve') or {}).get('reason') != 'observed_motion_no_entry_reserve'
            if len(changed) < 12:
                changed.append(dict(seq=b['seq'],time_s=b['reference_s'],before=a['requested'],after=b['requested']))
    return dict(input_sha256=old['input_sha256'], comparison=dict(counts),
        existing={k:old[k] for k in ('counts','capped_calls','processing_ms','source_unchanged')},
        candidate={k:new[k] for k in ('counts','capped_calls','processing_ms','source_unchanged')},
        proposals=[len(old['proposals']),len(new['proposals'])],examples=changed)


def closed():
    result = []
    for prefix in ('closed','static'):
        a,b = [read(BASE/f'{prefix}_{v}/report.json') for v in ('existing','motion_v1')]
        if not a['source_unchanged'] or not b['source_unchanged']:
            raise ValueError('source changed')
        for x,y in zip(a['cases'],b['cases']):
            if (x['case'],x['duration_s'],x['sensor_profile']) != (y['case'],y['duration_s'],y['sensor_profile']):
                raise ValueError('unmatched synthetic configuration')
            keys=('final_progress_m','final_lateral_m','contact_samples','max_wheels_touching_grass',
                  'min_gap_m','stopped','entry','returned_to_follow','reserve_applied_calls',
                  'reserve_zero_calls','mode_transitions','max_actual_speed_mps')
            result.append(dict(case=x['case'],duration_s=x['duration_s'],
                existing={k:x[k] for k in keys},candidate={k:y[k] for k in keys},
                progress_delta_m=y['final_progress_m']-x['final_progress_m'],
                replays=[x['schema5_replay'],y['offline_schema905_replay']]))
    return result


if __name__ == '__main__':
    result = dict(shadow=shadow())
    if (BASE/'static_motion_v1/report.json').exists():
        result['closed_loop'] = closed()
    (BASE/'comparison.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2,allow_nan=False))
