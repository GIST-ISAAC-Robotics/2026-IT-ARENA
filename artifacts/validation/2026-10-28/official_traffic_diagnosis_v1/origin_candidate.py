"""경로 표본이 아닌 선분 위 최근접점을 길이 원점으로 쓰는 비교 후보.

20 cm 한도·80 mm 경로 일치·25 cm 공동 지지·관측/점유/시간 검사는 그대로다.
운영 모듈은 수정하지 않는다. 이 파일을 import해도 monkey patch하지 않는다.
"""
import ast
from pathlib import Path
import zipfile
import numpy as np
from arena_autonomy import corner_corridor


def projected_origin_offset(path, nearest):
    """기존 최근접 꼭짓점의 앞/뒤 선분에서 실제 최근접점을 찾는다.

    반환값은 꼭짓점의 arc - 투영점의 arc. 뒤 표본에서 재기 시작하면 음수다.
    멀리 떨어진 다른 경로 가지로 원점을 옮기지 않는다.
    """
    p=np.asarray(path,dtype=float)
    lengths=np.linalg.norm(np.diff(p,axis=0),axis=1)
    arc=np.r_[0.,np.cumsum(lengths)]
    choices=[]
    for index in (nearest-1,nearest):
        if not 0 <= index < len(p)-1 or lengths[index] <= 1e-12:continue
        direction=p[index+1]-p[index]
        fraction=float(np.clip(-np.dot(p[index],direction)/(lengths[index]**2),0.,1.))
        point=p[index]+fraction*direction
        choices.append((float(np.dot(point,point)),index,fraction))
    if not choices:raise ValueError('no valid origin segment')
    _,index,fraction=min(choices)
    return float(arc[nearest]-(arc[index]+fraction*lengths[index]))


def baseline_source():
    with zipfile.ZipFile(Path(__file__).parent/'before_origin_fix.zip') as archive:
        text=archive.read('src/arena_autonomy/arena_autonomy/corner_corridor.py').decode('utf-8')
    node=next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name=='short_prefix_evidence')
    return ast.get_source_segment(text,node)+'\n'


def build_previous():
    namespace=dict(corner_corridor.__dict__)
    exec(compile(baseline_source(),'before_origin_fix.zip', 'exec'),namespace)
    return namespace['short_prefix_evidence']


def build_candidate():
    source=baseline_source()
    old="    first, end = int(starts[0]), int(ends[0])\n"
    new=old+"    origin_offset = projected_origin_offset(own, nearest)\n    lead_in = float(arc[first] + origin_offset)\n"
    assert source.count(old)==1
    source=source.replace(old,new)
    assert source.count('arc[first] > config.length_m')==1
    source=source.replace('arc[first] > config.length_m','not 0. <= lead_in <= config.length_m')
    assert source.count("'one_sided_lead_in_m':float(arc[first])")==1
    source=source.replace("'one_sided_lead_in_m':float(arc[first])",
        "'one_sided_lead_in_m':lead_in, 'origin_arc_correction_m':origin_offset")
    namespace=dict(corner_corridor.__dict__,projected_origin_offset=projected_origin_offset)
    exec(compile(source,'origin_candidate.py','exec'),namespace)
    return namespace['short_prefix_evidence']


short_prefix_evidence=build_candidate()
