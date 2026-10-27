"""원시 주행 결과의 실패/완주·정지·관측 사유를 따로 요약한다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    report_path=args.run/'report.json'
    trajectory=args.run/'trajectory.jsonl'
    raw=json.loads(report_path.read_text(encoding='utf-8'))
    rows=[json.loads(line) for line in trajectory.read_text(encoding='utf-8').splitlines()]
    changes=[];last=None
    for row in rows:
        safety=row.get('safety') or {}
        partial=safety.get('partial_observation') or {}
        key=(safety.get('reason'),partial.get('reason'),partial.get('latch_reason'))
        if key!=last:
            changes.append(dict(time_s=row['time'],progress_m=row['progress_m'],
                safety_reason=key[0],partial_reason=key[1],latch_reason=key[2],
                latch_time_s=partial.get('latch_time_s'),
                safe_speed_mps=safety.get('safe_speed_mps'),unknown_samples=partial.get('unknown_samples')))
        last=key
    result={k:raw.get(k) for k in ('passed','progress_m','completed_laps','collision_samples',
        'max_centerline_error_m','stop_stable','dense_pose_stop_evidence','failure_cleanup_stop',
        'source_unchanged','shutdown_clean','remaining_test_pids')}
    result.update(scope='summary_of_existing_samples_not_new_drive',
        source_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (report_path,trajectory,Path(__file__))},
        sample_count=len(rows),safety_counts=dict(Counter((r.get('safety') or {}).get('reason') for r in rows)),
        partial_counts=dict(Counter(((r.get('safety') or {}).get('partial_observation') or {}).get('reason') for r in rows)),
        state_changes=changes,error=raw.get('error'))
    with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps({k:result[k] for k in ('passed','progress_m','completed_laps','collision_samples','shutdown_clean','partial_counts')}))


if __name__=='__main__':main()
