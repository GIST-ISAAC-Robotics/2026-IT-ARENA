"""이미 저장한 양쪽 후보 반환의 도로 띠 비교. 벽 분류·실제 주행 허가 없음."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from observed_road_strip_probe import road_strip

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):parser.error('validation output required')
    out.mkdir(parents=True,exist_ok=False)
    files=[args.input,Path(__file__),ROOT/'scripts/observed_road_strip_probe.py',ROOT/'tests/test_observed_road_strip_probe.py']
    frozen={p.resolve().relative_to(ROOT).as_posix():p.read_bytes() for p in files}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for n,b in frozen.items():z.writestr(n,b)
    source=json.loads(args.input.read_text(encoding='utf-8'));rows=[]
    for case in source['cases']:
        rays=case.get('evaluation_selected_rays')
        if not rays:continue
        p=np.array([r['hit_xy_m'] for r in rays]);s=np.array([r['side'] for r in rays])
        # 기존 경로의 두 배 오차 띠로 고른 반환이다. 벽으로 검증된 입력이 아니다.
        probe=road_strip(p[s==1],p[s==-1],[0.,.1,.2,.3,.4,.5],slope_bound=math.tan(.08))
        rows.append(dict(bias_m=case['bias_m'],frame=case['frame'],probe=probe))
    unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    result=dict(cases=rows,inputs_unchanged=unchanged,source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        driving_authorized=False,scope='conditional same-side walls and <=0.08 rad tangent assumption; sampled cross sections only',
        no_truth_bias_removed=True,measured_bracket_limit_m=.1,geometry_parameters_adopted=False)
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    for r in rows:print(json.dumps({k:v for k,v in r.items()},ensure_ascii=False))
    return 0 if unchanged and len(rows)==4 else 1


if __name__=='__main__':raise SystemExit(main())
