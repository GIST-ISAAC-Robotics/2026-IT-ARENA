"""コ너 내부 HOLD 여부의 사전 등록 기준을 원본 평가와 별도로 계산한다."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from curve_stop_preflight import hold_geometry


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('run',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    assert not args.output.exists()
    names=('report.json','trace.json','ego_poses.jsonl','follow_decisions.jsonl')
    before={n:hashlib.sha256((args.run/n).read_bytes()).hexdigest() for n in names}
    report=json.loads((args.run/'report.json').read_text(encoding='utf8'))
    cfg=json.loads(Path('config/tests/g1_curve_stop_restart_v1.json').read_text(encoding='utf8'))
    assert report['case']=='curve_stop_restart'
    trace=json.loads((args.run/'trace.json').read_text(encoding='utf8'))
    poses=[json.loads(l) for l in (args.run/'ego_poses.jsonl').read_text(encoding='utf8').splitlines()]
    green=report['evaluation']['t_green']
    # 녹화 시작부터 yaw를 연속화하고 green의 직전 기준으로 정렬한다.
    yaws=np.unwrap([r[3] for r in poses])
    start=max(i for i,r in enumerate(poses) if r[0]<=green)
    hold=cfg['acceptance']['curve_stop_restart']['hold_window_s']
    result=hold_geometry(trace,poses,green+hold[0],green+hold[1],float(yaws[start]),cfg['acceptance']['curve_stop_geometry'])
    threats=[]
    for line in (args.run/'follow_decisions.jsonl').open(encoding='utf8'):
        e=json.loads(line)
        if e['kind']=='call':
            lateral=e['data']['output'][2].get('m3_lateral') or {}
            if any(v.get('present') for v in (lateral.get('threats') or {}).values()):
                threats.append(e['data']['input']['reference_s'])
    out={'scope':'post-hoc ground truth only, original verdict retained','original_passed':report['passed'],
        'original_run_valid':report['run_valid'],'curve_hold':result,'threat_calls':len(threats),
        'threat_span_s':[min(threats),max(threats)] if threats else None,'input_sha256':before,
        'inputs_unchanged':all(hashlib.sha256((args.run/n).read_bytes()).hexdigest()==v for n,v in before.items())}
    out['registered_combined_passed']=bool(report['passed'] and result['passed'] and result['all_hold_trace_poses_covered'])
    assert out['inputs_unchanged']
    args.output.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False))
