"""같은 78초 각본의 두 실제 실행 비교. 원시 실패는 그대로 나열한다."""
from pathlib import Path
import importlib.util,json
from hashlib import sha256

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
source=BASE.parent/'fast_lead_course_v1/summarize_trajectory.py'
spec=importlib.util.spec_from_file_location('trajectory_summary',source)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def extract(folder):
    run=folder/'run1';report=json.loads((run/'report.json').read_bytes())
    surface=json.loads((folder/'run1_surface.json').read_bytes())
    dense=json.loads((folder/'run1_dense_progress.json').read_bytes())
    trajectory=module.summarize(run)
    return dict(valid=report['run_valid'],source_unchanged=report['source_unchanged'],clean=report['shutdown_clean'],
        explicit_stop=all(report['stop_checks'].values()),failed_raw=[k for k,v in report['gates'].items() if not v],
        progress=report['evaluation']['progress'],gap=report['evaluation']['gap'],full_lap=report['gates']['full_lap'],
        maximum_off_wheels=surface['interpretations']['any']['max_off_wheels'],
        wall_wheel_samples=surface['wall_wheel_samples'],
        longest_grass_s=surface['interpretations']['any']['longest_continuous_grass_contact_s'],
        maximum_nonprogress=dense['max_nonprogress'],dense_window=dense['window'],trajectory=trajectory,
        report_sha256=sha256((run/'report.json').read_bytes()).hexdigest())


if __name__=='__main__':
    old=extract(BASE.parent/'fast_lead_course_v1');new=extract(BASE)
    assert all(new[k] for k in ('valid','source_unchanged','clean','explicit_stop'))
    with (BASE/'run1/follow_decisions.jsonl').open() as f:head=json.loads(next(f))['data']
    assert head['follow_config']['released_rear_sweep_trial'] is True
    value=dict(prior=old,candidate=new,
        scope='같은 78초 각본 각1회. 스캔/실행 위상은 다름. 단일 결과를 모든 상대와 실물 보장으로 확대하지 않음.')
    with (BASE/'comparison.json').open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2)
    for name,v in [('prior',old),('candidate',new)]:
        print(name,json.dumps(dict(progress=v['progress'],lap_s=v['trajectory']['lap_crossing_s'],
            nonprogress_s=v['maximum_nonprogress']['duration_s'],raw=v['failed_raw'],off_wheels=v['maximum_off_wheels']),ensure_ascii=False))
