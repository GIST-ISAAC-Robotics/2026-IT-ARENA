"""같은 네 합성 장면의 초기 선택만 진단 후보로 대조한다. 생산 코드 무수정."""
import hashlib
import json
from pathlib import Path
import zipfile
from analyze import BASE, ROOT
from prototype import rerank, choose_initial_intent
import arena_autonomy.bypass_entry_trial as entry_module
from probe_bypass_entry_sequence import rollout


original = entry_module._compare_entries
original_select = entry_module.EntryIntent.select
calls = []


def comparison(*args, **kwargs):
    result = original(*args, **kwargs)
    if kwargs.get('guide') is None and kwargs.get('stationary_boundaries'):
        result = rerank(args[0], result, kwargs['config'])
        calls.append(dict(reference_s=args[0].t_ref_s, selected=result['selected'],
                          diagnostic=result.get('prototype_ranking')))
    return result


if __name__ == '__main__':
    output = BASE/'prototype2'
    output.mkdir(exist_ok=False)
    sources = list((ROOT/'src').rglob('*.py'))+list(BASE.glob('*.py'))+list((ROOT/'scripts').glob('*bypass*.py'))
    sources += [ROOT/'scripts/g2_tracking_probe.py',ROOT/'scripts/g0_geometry.py',
                ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py']
    raw = {str(p.relative_to(ROOT)):p.read_bytes() for p in sources}
    with zipfile.ZipFile(output/'sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name,data in raw.items():
            archive.writestr(name,data)
    entry_module._compare_entries = comparison
    entry_module.EntryIntent.select = lambda self, scene, result: choose_initial_intent(self, scene, result, original_select)
    cases = []
    try:
        for target, restart in ((0.,False),(-.15,False),(.15,False),(0.,True)):
            begin = len(calls)
            result = rollout(target,restart,10.,True,True,True,True,'combined',17,True,True,False)
            result['prototype_initial_choices'] = calls[begin:]
            name = f'y{target}_restart{int(restart)}'
            (output/(name+'.json')).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
            cases.append({k:v for k,v in result.items() if k!='records'})
    finally:
        entry_module._compare_entries = original
        entry_module.EntryIntent.select = original_select
    report = dict(scope='initial tracking ranking prototype; synthetic sensors, ideal bicycle, same independent guard; not ROS/Gazebo or operating default',
                  cases=cases, source_sha256={n:hashlib.sha256(v).hexdigest() for n,v in raw.items()},
                  source_unchanged=all((ROOT/n).read_bytes()==v for n,v in raw.items()))
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(cases,indent=2))
    if not report['source_unchanged']:
        raise RuntimeError('source changed')
