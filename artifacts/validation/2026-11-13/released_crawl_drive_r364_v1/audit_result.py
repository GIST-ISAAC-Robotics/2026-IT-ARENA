"""원시 입력을 출력하지 않고 r364의 계획/현재 검사 정체를 요약한다."""
import collections
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent

def reasons(value, prefix=''):
    result = {}
    if isinstance(value, dict):
        for key, item in value.items():
            if key in ('input', 'scene', 'points', 'path_points'):
                continue
            name = prefix+'.'+key if prefix else key
            if isinstance(item, (dict, list)):
                result.update(reasons(item, name))
            elif key in ('reason','selector_mode','selector_choice','speed_mps','speed_cap_mps','valid','allowed'):
                result[name] = item
    elif isinstance(value, list) and len(value) == 3 and isinstance(value[2], dict):
        result[prefix+'.speed'] = value[0]
        result[prefix+'.steering'] = value[1]
        result.update(reasons(value[2], prefix+'.detail'))
    return result

def main():
    path = A/'run1/follow_decisions.jsonl'
    calls = [row['data'] for row in map(json.loads, path.open(encoding='utf-8')) if row['kind']=='call']
    statuses = list(map(json.loads, (A/'run1/autonomy_status.jsonl').open(encoding='utf-8')))
    end = [row for row in statuses if 23 < row.get('sim_time_s', 0) < 31]
    report = dict(original_passed=False, original_completion=False, source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  call_count=len(calls), stall_window_s=[23,31],
                  stall_status_counts=dict(collections.Counter(row['state'] for row in end)),
                  final_plan=reasons(calls[-1]['output'],'plan'),
                  final_current=reasons(calls[-1].get('current_rebase',{}),'current'),
                  policy_install=json.loads(next((A/'worker_policy_receipts').glob('*installed*.json')).read_text())
                    if list((A/'worker_policy_receipts').glob('*installed*.json')) else None,
                  scope='saved-run diagnosis, not another drive or successful completion')
    (A/'result_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False))

if __name__=='__main__': main()
