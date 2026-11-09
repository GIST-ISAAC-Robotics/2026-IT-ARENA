"""실행기 확장: 기존 상대차/접촉계를 보존하고 시설을 평가·렌더링에 함께 넣는다."""
import inspect,json
import xml.etree.ElementTree as ET
from pathlib import Path
from traffic_facility_fixture import merge
from replay_tools import load_launch
from build_experimental_track import rectangle
A=Path(__file__).resolve().parent

def configure(target,stage):
    stage=Path(stage);stage.mkdir(exist_ok=False)
    world=target.REPO/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
    tree=ET.parse(world)
    signal=load_launch()._signal_posts(tree.getroot().find('world'),'stl_61cm_provisional','official_v2026_11_06')
    marker=json.loads((A/'marker_posts_provisional.json').read_text(encoding='utf8'))
    posts=[rectangle(*p['center_xy_m'],signal['beam_yaw_rad'],p['along_m'],p['across_m'])
           for p in signal['links'] if p['name'].startswith('tl_post_')]
    posts += [rectangle(*p['center'],p['yaw'],p['length'],p['width']) for p in marker['links'] if p['part']=='post']
    original_obstacles=target.obstacle_polygons
    def obstacles(path):
        assert Path(path).resolve()==world.resolve()
        return [*original_obstacles(path),*posts]
    target.obstacle_polygons=obstacles
    original_start=target.start_demo_process
    def start(*args,**kwargs):
        bound=inspect.signature(original_start).bind(*args,**kwargs);bound.apply_defaults()
        extra=list(bound.arguments['extra_arguments'])
        indices=[i for i,a in enumerate(extra) if a.startswith('evaluation_actor_sdf:=')]
        assert len(indices)==1
        i=indices[0];actor=Path(extra[i].split(':=',1)[1]);combined=actor.with_name('evaluation_actors_and_marker_posts.sdf')
        evidence=merge(actor,combined)
        extra[i]='evaluation_actor_sdf:='+str(combined)
        added=['signal_posts:=stl_61cm_provisional','g1_narrow_occluder_bridge:=true',
               'g1_junction_continuity:=true','evaluation_course_markers:=blank_20_30']
        assert not any(a.split(':=')[0] in {v.split(':=')[0] for v in added} for a in extra)
        bound.arguments['extra_arguments']=[*extra,*added]
        evidence['added_launch_arguments']=added
        (actor.parent/'facility_merge.json').write_text(json.dumps(evidence,indent=2)+'\n')
        return original_start(*bound.args,**bound.kwargs)
    target.start_demo_process=start
    (stage/'facility_fixture.json').write_text(json.dumps(dict(signal=signal,marker=marker,
        collision_posts=len(posts),foot_height_evaluated_separately=True),indent=2)+'\n')
