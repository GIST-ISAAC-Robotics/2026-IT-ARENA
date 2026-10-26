"""기존 좌우 합성 시퀀스의 복귀 관문 ON/OFF. 자차 고정 개루프임을 유지한다."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/p) for p in ('src/arena_autonomy','src/arena_vehicle_interface','tests')]
from arena_autonomy.follow_trace import freeze
from arena_autonomy.m3_execution_trial import ExecutionConfig,M3ExecutionFollow
from arena_autonomy.m3_guarded_return_trial import GuardedReturnTrial
from test_m3_observed_follow import timed_scene
from test_m3_baseline_release import D0,car_x
from test_lateral_intent import car
from test_local_follow import history


def run(sign):
    controllers={'original':M3ExecutionFollow(execution=ExecutionConfig(release_to_baseline=True),allow_simulation_trial=True)}
    controllers.update({k:GuardedReturnTrial(execution=ExecutionConfig(release_to_baseline=True),enabled=on,allow_simulation_trial=True)
                        for k,on in [('off',False),('on',True)]})
    sequence=[(round(.4+.1*k,3),[]) for k in range(3)]
    sequence += [(round(.7+.1*k,3),[car(car_x(1.+.1*k),.13*sign,D0*sign)]) for k in range(20)]
    h=history(v=0.);rows=[];checks={'off_same':True,'positive_fallback_has_verified_envelope':True}
    for t,extra in sequence:
        scene,_=timed_scene(t,D0*sign,extra=extra)
        outputs={name:c.command(scene,h,0.,'left' if sign>0 else 'right') for name,c in controllers.items()}
        checks['off_same'] &= freeze(outputs['off'])==freeze(outputs['original'])
        row={'t':t}
        for name,out in outputs.items():
            lateral=out[2].get('m3_lateral') or {};gate=lateral.get('release_gate') or {}
            envelope=out[2].get('m3_envelope') or {}
            rejected=gate.get('reason')=='baseline_command_unconfirmed'
            if name=='on' and rejected and out[0]>0:
                checks['positive_fallback_has_verified_envelope'] &= bool(envelope.get('observation_ok') and envelope.get('geometry_ok'))
            row[name]={'speed':out[0],'steering':out[1],'reason':out[2]['reason'],
                'mode':lateral.get('mode'),'released':bool(lateral.get('released_to_baseline')),
                'baseline_rejected':rejected,'memory_confirmed':gate.get('memory_confirmed'),
                'envelope_observation_ok':envelope.get('observation_ok'),'envelope_geometry_ok':envelope.get('geometry_ok')}
        rows.append(freeze(row))
    return {'checks':checks,'rejected_baseline_calls':sum(r['on']['baseline_rejected'] for r in rows),
            'positive_verified_fallbacks':sum(r['on']['baseline_rejected'] and r['on']['speed']>0 for r in rows),
            'original_releases':sum(r['original']['released'] for r in rows),
            'on_releases':sum(r['on']['released'] for r in rows),'rows':rows}


def main(out):
    if out.exists():raise FileExistsError(out)
    files=list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))+[Path(__file__)]
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    results={'left':run(1.),'right':run(-1.)}
    checks={f'{side}_{key}':value for side,r in results.items() for key,value in r['checks'].items()}
    checks['left_intervention_exercised']=results['left']['rejected_baseline_calls']>0
    checks['left_existing_candidate_can_move']=results['left']['positive_verified_fallbacks']>0
    unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    out.mkdir(parents=True)
    report={'scope':'stationary ego synthetic sequence, no simulated plant', 'checks':checks,'source_sha256':hashes,
        'source_unchanged':unchanged,'results':results,
        'limits':['Right sequence need not release; existing asymmetry remains.',
                  'Positive output is not physical progress or proof of continuous safety.']}
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({s:{k:v for k,v in r.items() if k!='rows'} for s,r in results.items()}),flush=True)
    return 0 if unchanged and all(checks.values()) else 1


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);sys.exit(main(p.parse_args().output))
