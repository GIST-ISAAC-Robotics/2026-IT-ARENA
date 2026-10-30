"""직전 침범 비교의 동일 센서/제어 폐루프에 S자 합류 장면만 제공한다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import zipfile

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from scenario import Scenario
from arena_autonomy.follow_trace import replay_events,freeze

OLD=BASE.parent/'g2_intrusion_current_v1'
spec=importlib.util.spec_from_file_location('intrusion_baseline',OLD/'characterize.py')
baseline=importlib.util.module_from_spec(spec);spec.loader.exec_module(baseline)
baseline.Scenario=Scenario
baseline.CASES={
    'merge_before':dict(kind='s_merge',trigger_x=.95),
    'merge_alongside':dict(kind='s_merge',trigger_x=1.15),
}


def main():
    out=BASE/'run1';out.mkdir(exist_ok=False)
    files=[*BASE.glob('*.py'),*BASE.glob('*.sh'),BASE/'PLAN.md',OLD/'characterize.py']
    files += [p for directory in ('src/arena_autonomy/arena_autonomy','src/arena_vehicle_interface/arena_vehicle_interface','scripts')
              for p in (ROOT/directory).glob('*.py')]
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for path in hashes:archive.write(ROOT/path,path)
    reports=[]
    for name in baseline.CASES:
        trace=(out/(name+'.jsonl')).resolve()
        summary,records=baseline.rollout(name,'observed_entry_space_v1',trace,duration=16.,
            window='trim_before_compare_v1',reserve=True,reacquire='known_hold_v1')
        with trace.open() as stream:replay=replay_events(json.loads(line) for line in stream)
        summary['replay']={k:v for k,v in replay.items() if k not in ('outcomes','mismatches')}
        summary['mismatches']=len(replay['mismatches'])
        summary['controller_read_actor_truth']=False
        summary['evaluation_target_speed_mps']=None  # 변속 궤적은 evaluation_scenario에 명시한다.
        summary['case_drive_passed']=None
        (out/(name+'.records.json')).write_text(json.dumps(freeze(records))+'\n')
        reports.append(summary);print(json.dumps(summary),flush=True)
    result=dict(reports=reports,scope='kinematic S-merge actor, sequential sensors, ideal bicycle and independent guard function',
        source_sha256=hashes,source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==v for p,v in hashes.items()))
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0 if result['source_unchanged'] and all(r['mismatches']==0 for r in reports) else 1


if __name__=='__main__':raise SystemExit(main())
