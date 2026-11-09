"""E1: 신호+마커 임시 지주와 마커20/30 미검출을 함께 둔 본선1랩.
현재 D3와 같은 속도·제어·물성. 시설은 기존 평가 모델 삽입 인터페이스만 사용.
"""
import inspect,json,sys,hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'scripts'),str(R/'tests')]
import validate_basic_autonomy as target
from run_new_course_solo import preset_args
fixture=A/'marker_posts_provisional.sdf'
meta=json.loads((A/'marker_posts_provisional.json').read_text())
out=A/'e_all_posts_blank';assert not out.exists()
assert hashlib.sha256(fixture.read_bytes()).hexdigest()==meta['fixture_sha256']
posts=[];feet=[]
for row in meta['links']:
    poly=target.rectangle(*row['center'],row['yaw'],row['length'],row['width'])
    (posts if row['part']=='post' else feet).append(poly)
original_start=target.start_demo_process
original_obstacles=target.signal_post_obstacles

def start(*args,**kwargs):
    b=inspect.signature(original_start).bind(*args,**kwargs);b.apply_defaults()
    b.arguments['extra_arguments']=[*b.arguments['extra_arguments'],'evaluation_actor_sdf:='+str(fixture)]
    return original_start(*b.args,**b.kwargs)

def obstacles(*args,**kwargs):
    all_obstacles,signal_posts,signal_feet,report=original_obstacles(*args,**kwargs)
    return [*all_obstacles,*posts],signal_posts,signal_feet,report

target.start_demo_process=start;target.signal_post_obstacles=obstacles
sys.argv=preset_args('new-course-solo',out)+['--signal-posts','stl_61cm_provisional',
    '--g1-narrow-occluder-bridge','--g1-junction-continuity','--course-marker-visibility','blank_20_30']
(A/'e_args.json').write_text(json.dumps(dict(args=sys.argv,fixture=str(fixture),fixture_sha256=meta['fixture_sha256']),indent=2)+'\n')
code=target.main()
# 보충 시설 실제 삽입·정지구간 포함 별도 평가. 원 report 판정은 보존한다.
if (out/'runtime_world.sdf').exists():
    runtime=ET.parse(out/'runtime_world.sdf').getroot().find('world')
    expected=ET.parse(fixture).getroot().findall('model')
    applied=all(runtime.find(f"model[@name='{m.get('name')}']") is not None for m in expected)
    identical=all(ET.canonicalize(ET.tostring(runtime.find(f"model[@name='{m.get('name')}']"),encoding='unicode'),strip_text=True)==
        ET.canonicalize(ET.tostring(m,encoding='unicode'),strip_text=True) for m in expected) if applied else False
    trace=json.loads((out/'trace.json').read_text()) if (out/'trace.json').exists() else []
    stop=json.loads((out/'stop_trace.json').read_text()) if (out/'stop_trace.json').exists() else []
    result=dict(applied=applied,model_xml_identical=identical,fixture_unchanged=hashlib.sha256(fixture.read_bytes()).hexdigest()==meta['fixture_sha256'],
        drive=target.signal_post_clearance(trace,posts,feet),stop=target.signal_post_clearance(stop,posts,feet),
        scope=meta['scope'])
    (out/'marker_post_evidence.json').write_text(json.dumps(result,indent=2)+'\n')
raise SystemExit(code)
