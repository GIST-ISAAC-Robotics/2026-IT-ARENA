"""실제로 실행한 대표의 기능 조합을 구분한다. 서로 다른 옵션의 성과를 합산하지 않는다."""
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
FIELDS=('piecewise_wall_trial','narrow_occluder_bridge','junction_continuity',
        'standing_wall_reacquire','traffic_boundary_refresh_trial','traffic_recovery_trial',
        'guard_curve_trial','current_lane_preference_trial')
RUNS=dict(
    two_actor_facilities=A/'two_actor_run1/run1',
    stationary_facilities_r60=A/'stationary_r60/run1',
    recessed_facilities_r60=A/'stationary_recessed_r60/run1')


def main():
    out=A/'option_coverage.json';assert not out.exists()
    rows={}
    for name,path in RUNS.items():
        with (path/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
            event=json.loads(next(stream))
        assert event['kind']=='header'
        h=event['data']
        report=json.loads((path/'report.json').read_bytes())
        f=h['follow_config'];e=h['execution_config']
        rows[name]=dict(run=path.relative_to(ROOT).as_posix(),
            header_sha256=hashlib.sha256(json.dumps(h,sort_keys=True).encode()).hexdigest(),
            follow_config={k:f[k] for k in FIELDS},moving_pass_profile=e['moving_pass_profile'],
            actual_sources={k:v for k,v in report['source_sha256'].items()
                if k.endswith(('pass_release_trial.py','local_follow.py'))},
            passed_original=report['passed'],run_valid=report['run_valid'],
            source_unchanged=report['source_unchanged'])
    output=dict(scope='executed header/source coverage only; not combined behavior proof',runs=rows)
    out.write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:dict(**v['follow_config'],moving_pass_profile=v['moving_pass_profile']) for k,v in rows.items()}))


if __name__=='__main__':main()
