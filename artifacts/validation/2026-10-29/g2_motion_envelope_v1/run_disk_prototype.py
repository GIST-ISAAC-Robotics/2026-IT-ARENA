"""고정55 mm/기존 표본 방식에서 이동 도형만 바꾼 중앙 복합 조건1회."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from rounded_sweep import sampled_rounded_limit
import arena_autonomy.bypass_entry_trial as entry_module
from probe_bypass_entry_sequence import rollout

if __name__=='__main__':
    out=BASE/'disk1';out.mkdir(exist_ok=False)
    paths=list((ROOT/'src').rglob('*.py'))+list(BASE.glob('*.py'))+list((ROOT/'scripts').glob('*bypass*.py'))
    paths += [ROOT/'scripts/g2_tracking_probe.py',ROOT/'scripts/g0_geometry.py',
              ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py']
    raw={str(p.relative_to(ROOT)):p.read_bytes() for p in paths}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for n,v in raw.items():z.writestr(n,v)
    original=entry_module.swept_limit
    entry_module.swept_limit=sampled_rounded_limit
    try:
        result=rollout(0.,False,10.,True,True,True,True,'combined',17,True,True,False)
    finally:
        entry_module.swept_limit=original
    (out/'case.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    report=dict(scope='synthetic central stopped actor, combined sensors; changed displacement geometry only; no production connection',
                result={k:v for k,v in result.items() if k!='records'},
                source_sha256={n:hashlib.sha256(v).hexdigest() for n,v in raw.items()},
                source_unchanged=all((ROOT/n).read_bytes()==v for n,v in raw.items()))
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report['result'],indent=2))
    if not report['source_unchanged']:raise RuntimeError('source changed')
