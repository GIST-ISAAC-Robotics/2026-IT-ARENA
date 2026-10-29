"""개방형 표적의 합성 센서/자동 G1-G2 폐루프. 원래 충돌 외형 유지."""
import hashlib
from functools import partial
import importlib.util
import json
import math
from pathlib import Path
import sys
import zipfile

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from g0_geometry import TruthBox,Footprint
spec=importlib.util.spec_from_file_location('reserved_handoff_probe',BASE.parent/'g2_follow_reserve_v1/probe_closed_loop.py')
common=importlib.util.module_from_spec(spec);spec.loader.exec_module(common)


def open_parts(target):
    """센서 광선용6부품. 제어기에는 형상/정답을 전달하지 않는다."""
    x,y,a=target.x_m,target.y_m,target.yaw_rad
    parts=[TruthBox(x,y,.02,target.footprint,a,0.,'target_bottom'),
           TruthBox(x,y,.02,target.footprint,a,.12,'target_top')]
    for dx in (-.08,.08):
        for dy in (-.055,.055):
            parts.append(TruthBox(x+dx*math.cos(a)-dy*math.sin(a),y+dx*math.sin(a)+dy*math.cos(a),
                .10,Footprint(.02,.02),a,.02,'target_post'))
    return parts


original_render=common.probe.render_scan


def render_open(rows,now,target_y=0.,restart=False,sensors=None):
    physical_boxes=common.probe.evaluation_boxes
    def sensor_boxes(at,y=0.,restart=False):
        boxes=physical_boxes(at,y,restart)
        return boxes[:-1]+open_parts(boxes[-1])
    # 평가 함수는 원래 전체20x15cm 외형을 사용한다. 상하판 전체 투영과 같다.
    common.probe.evaluation_boxes=sensor_boxes
    try:return original_render(rows,now,target_y,restart,sensors)
    finally:common.probe.evaluation_boxes=physical_boxes


def main():
    policy='contiguous_three_v0'
    if '--patch-policy' in sys.argv:
        pos=sys.argv.index('--patch-policy');policy=sys.argv[pos+1]
        del sys.argv[pos:pos+2]
    common.HandoffExecutionConfig=partial(common.HandoffExecutionConfig,front_patch_policy=policy)
    idx=sys.argv.index('--output')+1;out=BASE/sys.argv[idx]
    own={str(Path(__file__).relative_to(ROOT)):Path(__file__).read_bytes()}
    common.BASE=BASE
    common.probe.render_scan=render_open
    common.main()
    report=json.loads((out/'report.json').read_text())
    with zipfile.ZipFile(out/'sources.zip','a',zipfile.ZIP_DEFLATED) as z:
        for n,v in own.items():z.writestr(n,v)
    report['shape']='6-part open frame; full 20x15cm collision footprint retained'
    report['front_patch_policy']=policy
    report['additional_source_sha256']={n:hashlib.sha256(v).hexdigest() for n,v in own.items()}
    report['source_unchanged']=report['source_unchanged'] and all((ROOT/n).read_bytes()==v for n,v in own.items())
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    if not report['source_unchanged']:raise RuntimeError('shape source changed')


if __name__=='__main__':main()
