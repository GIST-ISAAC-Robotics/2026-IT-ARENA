"""새 도형 표현과 보존 결과의 수치·판정 비교. 주행/추종 재실행 없음."""
from pathlib import Path
import hashlib
import json
import math
import warnings
import zipfile

from g0_geometry import TruthBox
from g2_geometry_probe import ProbeConfig, sample_path
from g2_surface_geometry import assess
from g2_surface_tracking import inspect_rollout

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
OUT = HERE/'run3_component_compare'/'data'


def compare(old, new, path=''):
    if isinstance(old, dict):
        assert old.keys() == new.keys(), path
        return max((compare(v, new[k], path+'/'+k) for k,v in old.items()), default=0.)
    if isinstance(old, (list, tuple)):
        assert len(old) == len(new), path
        return max((compare(a,b,path+f'/{i}') for i,(a,b) in enumerate(zip(old,new))), default=0.)
    if isinstance(old, bool) or old is None or isinstance(old, str):
        assert old == new, (path,old,new)
        return 0.
    assert math.isfinite(old) and math.isfinite(new), path
    assert math.isclose(old,new,rel_tol=0.,abs_tol=1e-12), (path,old,new)
    return abs(old-new)


def main():
    OUT.mkdir(parents=True,exist_ok=False)
    names = ['scripts/g2_surface_geometry.py','scripts/g2_surface_tracking.py',
             'scripts/g2_geometry_probe.py','scripts/g2_tracking_probe.py',
             'scripts/g0_geometry.py','scripts/surface_contact_eval.py',
             'src/arena_autonomy/arena_autonomy/lateral_intent.py',
             'tests/test_g2_surface_geometry.py','tests/test_g2_surface_tracking.py']
    raw={p:(ROOT/p).read_bytes() for p in names}
    raw[str(Path(__file__).resolve().relative_to(ROOT))]=Path(__file__).read_bytes()
    inputs=[HERE/'run1/data/report.json',HERE/'run2_tracking/data/report.json']
    inputs+=sorted((HERE/'run2_tracking/data').glob('*_rollout.json'))
    before={str(p.relative_to(ROOT)):p.read_bytes() for p in inputs}
    with zipfile.ZipFile(OUT/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p,b in raw.items(): z.writestr(p,b)
    cases=[]
    with warnings.catch_warnings():
        warnings.simplefilter('error',RuntimeWarning)
        report=json.loads(inputs[0].read_text())
        for group in report['results']:
            target=TruthBox(**group['target']) if not isinstance(group['target'].get('footprint'),dict) else None
            if target is None:
                from g0_geometry import Footprint
                fields={**group['target'],'footprint':Footprint(**group['target']['footprint'])}
                target=TruthBox(**fields)
            for old in group['candidates']:
                path=sample_path(old['offset_m'],old['transition_m'],ProbeConfig(step_m=.001))
                new=assess(path,target)
                cases.append(dict(kind='geometry',target_y_m=target.y_m,offset_m=old['offset_m'],
                                  transition_m=old['transition_m'],maximum_absolute_delta=compare(old,new),
                                  conditions_met=new['geometry_conditions_met']))
        report=json.loads(inputs[1].read_text())
        for old in report['results']:
            rows=json.loads((HERE/'run2_tracking/data'/(old['name']+'_rollout.json')).read_text())
            new=inspect_rollout(rows,TruthBox(1.5,.15,.12))
            cases.append(dict(kind='tracking_evaluation',name=old['name'],
                              maximum_absolute_delta=compare(old['surface'],new),checks=new['checks']))
    source_unchanged=all((ROOT/p).read_bytes()==b for p,b in raw.items())
    input_unchanged=all((ROOT/p).read_bytes()==b for p,b in before.items())
    result=dict(cases=cases,source_unchanged=source_unchanged,input_unchanged=input_unchanged,
                source_sha256={p:hashlib.sha256(b).hexdigest() for p,b in raw.items()},
                input_sha256={p:hashlib.sha256(b).hexdigest() for p,b in before.items()},
                runtime_warnings_are_errors=True,comparison_absolute_tolerance=1e-12,
                driving_authorized=False,controller_rollouts_rerun=False)
    (OUT/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    print(json.dumps(dict(cases=len(cases),max_delta=max(r['maximum_absolute_delta'] for r in cases),
                          source_unchanged=source_unchanged,input_unchanged=input_unchanged)))
    return 0 if source_unchanged and input_unchanged else 1


if __name__=='__main__':
    raise SystemExit(main())
