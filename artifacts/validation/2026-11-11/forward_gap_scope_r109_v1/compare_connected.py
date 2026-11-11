"""r109 전방 폭의 범위만 수정한 전후 판단 비교. 원 궤적 고정, 주행 증명 아님."""
import hashlib
import json
import sys
import types
import zipfile
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision


def replay(rows):
    follow,motion=make_controller(rows[0]['data'])
    outputs=[]
    for row in rows[1:]:
        if row['kind']=='reset':follow.reset()
        elif row['kind']=='call':
            outputs.append((row,freeze(replay_call(follow,row['data']['input'],motion))))
    return outputs


def main():
    predecessor=types.ModuleType('arena_autonomy.partial_r107')
    predecessor.__package__='arena_autonomy'
    with zipfile.ZipFile(A/'before.zip') as z:
        source=z.read('src/arena_autonomy/arena_autonomy/partial_observation.py')
    exec(compile(source,'partial_r107.py','exec'),predecessor.__dict__)
    from arena_autonomy import same_path_gap_trial as gap
    with zipfile.ZipFile(A/'before.zip') as z:
        old_gap=types.ModuleType('arena_autonomy.same_path_r107')
        old_gap.__package__='arena_autonomy'
        exec(compile(z.read('src/arena_autonomy/arena_autonomy/same_path_gap_trial.py'),'gap_r107.py','exec'),old_gap.__dict__)
    current=PartialObservationPolicy.evaluate
    current_gap=gap.bounded_same_path_gap
    result=[]
    for relative in ('scan_start_phase_r107_v1/two_actor_run1/run1/follow_decisions.jsonl',
                     'control_10hz_r91_v1/two_actor_run4/run1/follow_decisions.jsonl'):
        path=A.parent/relative
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        rows=[json.loads(line) for line in path.open(encoding='utf8')]
        PartialObservationPolicy.evaluate=predecessor.PartialObservationPolicy.evaluate
        gap.bounded_same_path_gap=old_gap.bounded_same_path_gap
        try:baseline=replay(rows)
        finally:
            PartialObservationPolicy.evaluate=current
            gap.bounded_same_path_gap=current_gap
        actual=replay(rows)
        mismatches=[row['seq'] for row,out in baseline if not equal_output(
            comparable_decision(out),comparable_decision(row['data']['output']))]
        if relative.startswith('scan_start_phase_r107'):assert not mismatches,mismatches[:10]
        changes=[]
        for (row,old),(same,new) in zip(baseline,actual):
            assert row['seq']==same['seq']
            if equal_output(old[:2],new[:2]):continue
            n=new[2].get('normal',new[2])
            changes.append(dict(seq=row['seq'],reference_s=row['data']['input']['reference_s'],
                before=old[:2],after=new[:2],reason=n.get('reason'),selector=n.get('selector_reason'),
                partial=n.get('partial_observation'),same_path=n.get('selector_same_path_gap')))
        assert hashlib.sha256(path.read_bytes()).hexdigest()==digest
        result.append(dict(source=path.relative_to(ROOT).as_posix(),sha256=digest,calls=len(actual),
                           baseline_mismatches=mismatches,changes=changes,
                           last=[dict(seq=row['seq'],output=out[:2],detail=out[2]) for row,out in actual[-1:]]))
    with (A/'comparison_connected.json').open('x',encoding='utf8') as f:json.dump(result,f,indent=2)
    print(json.dumps([dict(source=r['source'],calls=r['calls'],baseline_mismatches=len(r['baseline_mismatches']),
        command_changes=len(r['changes']),zero_to_positive=sum(x['before'][0]==0 and x['after'][0]>0 for x in r['changes']),
        positive_to_zero=sum(x['before'][0]>0 and x['after'][0]==0 for x in r['changes']),
        last_output=r['last'][0]['output']) for r in result]))


if __name__=='__main__':main()
