"""직전 실제 주행과 같은 제어/물성/오차, 새 평가용 합류 궤적만 등록한다."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
PARENT=BASE.parent/'g2_handoff_corridor_gazebo_v1'
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from merge_actor import MergeActor
import g1_traffic_eval as ev
from build_experimental_track import obstacle_polygons
from audit_local_pursuit_road import road_polygon


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    old=json.loads((PARENT/'run1_prepared.json').read_text())
    relevant={p:h for p,h in old['source_sha256'].items() if p.startswith(('src/','scripts/','config/','tests/'))}
    cfg=json.loads((BASE/'g2_center_config_v1.json').read_text());previous=json.loads((PARENT/'g2_center_config_v1.json').read_text())
    installed=ROOT/'install/arena_autonomy/lib/python3.12/site-packages/arena_autonomy'
    names=('bypass_handoff.py','g1_g2_node.py','bypass_handoff_corridor_trial.py','bypass_signed_hold_trial.py')
    world=ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture'
    data=np.genfromtxt(world/'centerline.csv',delimiter=',',names=True)
    scene=json.loads((world/'scene.json').read_text())
    centerline=ev.Centerline(data['x_m'],data['y_m'],data['s_m'],scene['track']['lap_length_m'])
    slot=next(s for s in scene['starting_grid']['slots'] if s['index']==cfg['launch']['grid_slot'])
    s0=slot['s_m']+cfg['cases'][0]['actor_start_center_gap_m']
    trial=MergeActor();route=trial.validate_route(cfg,centerline,cfg['cases'][0],s0,
        road_polygon(world/'world.sdf'),obstacle_polygons(world/'world.sdf'))
    checks=dict(parent_preflight_passed=json.loads((PARENT/'preflight.json').read_text())['passed'],
        controller_physics_sources_unchanged=all((ROOT/p).is_file() and sha(ROOT/p)==h for p,h in relevant.items()),
        installed_current=all(sha(installed/n)==sha(ROOT/'src/arena_autonomy/arena_autonomy'/n) for n in names),
        same_launch_actor_seeds=all(cfg[k]==previous[k] for k in ('launch','actor','seeds')),
        same_sensor_stress=all(sha(BASE/p)==sha(PARENT/p) for p in ('sensor_stress.py','range_profile.json','simulation.launch.py')),
        same_execution_header=cfg['acceptance']['g2']['trace_header_expected']==previous['acceptance']['g2']['trace_header_expected'],
        same_stop_and_tracking=all(cfg['acceptance'][k]==previous['acceptance'][k] for k in ('stop','harness')),
        unchanged_synthetic_scenario=sha(BASE/'scenario.py')==sha(BASE.parent/'g2_merge_departure_v1/scenario.py'),
        actor_route=route['wall_clear'],trigger_not_consumed=trial.scenario.start is None,
        dynamic_transport=cfg['cases'][0]['schedule']==[dict(from_s=0.,to_s=20.,speed_mps=0.)])
    result=dict(passed=all(checks.values()),checks=checks,route=route,scope='static source and evaluation trajectory preflight; no driving result')
    with (BASE/'preflight.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
