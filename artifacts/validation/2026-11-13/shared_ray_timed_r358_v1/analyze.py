"""제한 시간 재생 결과만 요약한다. 파일 발행을 실제 ROS/차량 구동으로 바꾸지 않는다."""
from pathlib import Path
import json,statistics

A=Path(__file__).resolve().parent

def stats(values):
    if not values:return None
    v=sorted(values)
    return dict(n=len(v),median_ms=statistics.median(v),p95_ms=v[int((len(v)-1)*.95)],max_ms=v[-1])

def main():
    rows=[]
    for folder in sorted((A/'jetson_results').glob('jetson_*')):
        if not folder.is_dir():continue
        r=json.loads((folder/'report.json').read_bytes())
        events=[json.loads(s) for s in (folder/'events.jsonl').read_text().splitlines()]
        stages={stage:stats([x['elapsed_ms'] for x in events if x['kind']=='completed' and x['stage']==stage]) for stage in ('plan','current')}
        rows.append(dict(run=folder.name,classification=r['classification'],published=r['published'],positive=r['positive'],
            retained_positive=r['retained_positive'],positive_gap=r['gaps'],previous_command_age=r['previous_command_age_before_next'],
            expired_positive_intervals=r['expired_positive_intervals'],expired_positive_with_tail_s=r['expired_positive_with_tail_s'],
            stage_cost=stages,discards=r['discards'],cleanup=r['cleanup'],trace_sha256=r['trace_sha256']))
    execution=json.loads((A/'jetson_results/execution.json').read_bytes())
    summary=dict(runs=rows,scope='Two compute processes; recorded sensor trajectory; file-only publication; not ROS or driving proof',
        all_clean=len(rows)==4 and all(not c['forced'] and c['exitcode']==0 for r in rows for c in r['cleanup']),
        changed_sources=execution['changed_sources'],execution_error=execution['error'],
        source_gap_preserved=json.loads((A/'input_preflight.json').read_bytes()))
    (A/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary))
    assert summary['all_clean'] and not summary['changed_sources'] and summary['execution_error'] is None

if __name__=='__main__':main()
