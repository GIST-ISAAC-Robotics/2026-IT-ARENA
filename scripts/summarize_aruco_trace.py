"""내부 후보 추적 근거를 재집계합니다. 기존 결과·검출기는 변경하지 않습니다."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-09-15/official_update'


def main():
    source=ROOT/'aruco_internal_trace.json'
    dest=ROOT/'aruco_internal_summary.json'
    if dest.exists(): raise FileExistsError(dest)
    data=json.loads(source.read_text())
    result={'comparisons': data['comparisons'], 'builtin_matches': data['builtin_matches'],
            'cases': {}, 'raw_id30_distance_examples': []}
    for case in ('raw_route','face_compare_pbr','face_compare_cells'):
        rows=[t for t in data['traces'] if t['case']==case and t['rate']==.05]
        out=[]
        for t in rows:
            if t['target'] in t['builtin_ids']: continue
            readable=[c['index'] for c in t['candidates'] if c['standalone_id']==t['target']]
            groups=[]
            for g in t['groups']:
                lost=set(g['members'])&set(readable)
                if lost and g['decoded_id']!=t['target']:
                    kept=t['candidates'][g['kept']]
                    groups.append({'members':g['members'],'readable_discarded':sorted(lost),
                        'kept':g['kept'],'kept_window':kept['window'],
                        'kept_border_errors':kept['border_errors'], 'allowed':kept['border_error_limit'],
                        'kept_corners':kept['corners']})
            out.append({'view':t['view'],'readable_before_grouping':readable,
                        'loss_explained_by_grouping':bool(groups),'groups':groups})
        result['cases'][case]={'misses':len(out),
            'grouping_explained':sum(v['loss_explained_by_grouping'] for v in out),'details':out}
    for t in data['traces']:
        if t['case']=='raw_route' and t['target']==30 and t['rate']==.05:
            valid=[c for c in t['candidates'] if c['standalone_id']==30]
            groups=[g for g in t['groups'] if set(g['members'])&{c['index'] for c in valid}]
            result['raw_id30_distance_examples'].append({'view':t['view'],'detected':30 in t['builtin_ids'],
                'readable_candidates':valid,'related_groups':groups,
                'selected_candidates':[t['candidates'][g['kept']] for g in groups]})
    dest.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='raw_id30_distance_examples'},ensure_ascii=False,indent=2))


if __name__=='__main__': main()
