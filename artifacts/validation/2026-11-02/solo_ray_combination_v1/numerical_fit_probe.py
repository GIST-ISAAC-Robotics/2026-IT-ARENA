"""기록된 첫 래치의 표면 대응 수치 수렴 비교. 런타임/판정선은 변경하지 않는다."""
from pathlib import Path
import inspect, json, math, sys, time
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
import arena_autonomy.partial_surface_alignment as original
from arena_autonomy.follow_trace import thaw

if __name__=='__main__':
    first=None
    for line in (HERE/'run1/follow_decisions.jsonl').open():
        e=thaw(json.loads(line))
        if e['kind']!='call':continue
        info=e['data']['output'][2]
        p=info.get('partial_observation') or {}
        if p.get('latch_evidence'):
            first=e['data'];break
    assert first is not None
    part=first['output'][2]['partial_observation']
    evidence=part['latch_evidence'];ctx=evidence['correspondence_context']
    source=inspect.getsource(original.surface_translation)
    variants={'original':source,'iterations12':source.replace('range(6)','range(12)'),
              'iterations24':source.replace('range(6)','range(24)'),
              'damped12':source.replace('range(6)','range(12)').replace('shift+=step','shift+=.5*step'),
              'damped24':source.replace('range(6)','range(24)').replace('shift+=step','shift+=.5*step')}
    result=dict(scope='첫 래치 좌표의 수치 탐색만 수행. 주행/제어기 변경/오류 해소 증명 아님.',reference_s=first['input']['reference_s'],latch_time_s=part['latch_time_s'],variants={})
    for name,code in variants.items():
        space=dict(original.__dict__);exec(code,space)
        value=space['surface_translation'](ctx['previous_points_now_xy'],ctx['current_return_points_xy'],ctx['current_return_source_indices'])
        result['variants'][name]=value
    (HERE/'numerical_fit_probe.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:dict(accepted=v['accepted'],reason=v['reason'],steps=len(v.get('steps',[])),last=v.get('steps',[None])[-1],translation=v.get('translation_m')) for k,v in result['variants'].items()},ensure_ascii=False))
