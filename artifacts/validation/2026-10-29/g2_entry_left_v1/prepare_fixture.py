"""G2 왼쪽 가장자리 정지 표적 1회 시험(run1 오른쪽의 대칭)의 설정·launch 사본을 실행 전에 고정한다.

g2_entry_gazebo_v1/prepare_fixture.py를 복사해 표적 부호(+0.149 m)·case 이름·config 이름·acceptance.g2 v2만 바꿨다.
launch 사본은 같은 방식으로 다시 만들고 run1 fixture와 바이트 동일한지 preflight에서 확인한다.

공식 원본·production launch·기존 v2 신호 시험본·5랩 소스는 수정하지 않는다. 월드는 5랩에서 쓴
v2 신호 시험본을 그대로 참조하고, 이 시험의 launch 사본만 (1) 월드 경로 (2) 뒤쪽30 cm 시험 카메라
(3) 실행 파일 g2_entry_trial (4) 도로 중심선 시작 자세를 바꾼다.
"""
import difflib
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
SIGNAL=ROOT/'artifacts/validation/2026-10-27/official_departure_v2'
WORLD=SIGNAL/'fixture'
BASE_CONFIG=ROOT/'config/tests/g1_traffic_v1.json'
sys.path.insert(0,str(ROOT/'scripts'))
import g1_traffic_eval as ev

GRID_SLOT=3
TARGET_GAP_M=1.2        # 합성 sequence의 표적 중심 x=1.2 m(자차 중심 기준)
TARGET_LATERAL_M=+.149  # 진행 방향 왼쪽 가장자리 표적(run1 −0.149 m의 대칭). 자차는 오른쪽 공간으로 우회해야 한다.
# run1: -.150은 공식 도로 다각형(중심선 기준 약0.02 mm 안쪽)을 3.9e-6 m2 넘어 기존 경로 검사에서 거절됐다(attempt1_lateral150).
DURATION_S=12.0
CASE='g2_edge_stopped_left'
CONFIG_NAME='g2_config_v2_left.json'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_new(path,text):
    with path.open('x',encoding='utf-8',newline='\n') as stream:stream.write(text)


def main():
    out=BASE/'fixture';out.mkdir(exist_ok=False)
    scene=json.loads((WORLD/'scene.json').read_text(encoding='utf-8'))
    route=np.genfromtxt(WORLD/'centerline.csv',delimiter=',',names=True)
    centerline=ev.Centerline(route['x_m'],route['y_m'],route['s_m'],scene['track']['lap_length_m'])
    slot={int(s['index']):s for s in scene['starting_grid']['slots']}[GRID_SLOT]
    x,y,yaw=centerline.pose(slot['s_m'])
    spawn=dict(x=float(x),y=float(y),yaw_rad=float(yaw),s_m=float(slot['s_m']),
               lateral_shift_from_grid_slot_m=float(math.hypot(slot['x']-x,slot['y']-y)))
    config=json.loads(BASE_CONFIG.read_text(encoding='utf-8'))
    config.update(
        registered_at='2026-10-29 before the G2 left-edge Gazebo run (run1 right-edge symmetric)',
        scope=('G2 explicit sensor entry trial: one stationary 20x15x12 cm box on the LEFT road edge (mirror of run1), ego starts on the '
               'centerline of the official start straight, official 4-red visual start, rendered sequential C1, '
               'independent LiDAR safety, virtual MCU. Not automatic race-mode selection, center target, hidden '
               'rear car, irregular vehicle shape, hardware or full-lap evidence.'),
        derived_from={'path':str(BASE_CONFIG.relative_to(ROOT)),'sha256':sha(BASE_CONFIG)})
    config['launch']['grid_slot']=GRID_SLOT
    config['launch']['g2_spawn']=spawn
    config['actor'].update(start_center_gap_m=TARGET_GAP_M,lateral_offset_m=0.,
        start_center_gap_note='target centre 1.2 m ahead of ego centre along the centerline (synthetic sequence layout)')
    config['cases']=[dict(name=CASE,behavior_kind='stopped_center',route_kind='side_road_straight',
        duration_s=DURATION_S,schedule=[],actor_lateral_offset_m=TARGET_LATERAL_M,actor_start_center_gap_m=TARGET_GAP_M,
        expected=('G2 enters the right space, passes the stationary left-edge target and returns to the centre. '
                  'The preserved G1 stopped_center gates are expected to fail by design (no HOLD, no stay-behind).'))]
    config['acceptance'][CASE]=dict(config['acceptance']['stopped_center'])
    config['acceptance']['g2']=dict(
        registered_before_run=True,
        version='g2_acceptance_v2',
        registered_note=('v2 = run1 registered gates + window split + negative-tested core (g2_eval_core.py). Applied to run1 only '
                         'as a labelled post-hoc audit, never as run1 registered criteria.'),
        target_passed='ego body rear corner passes the target body front corner along the centerline (s), before window end',
        windows=('drive window = [green, case_window end]; pass/return/hold judged only there. STOP window = (window end, stop '
                 'trace end] gated separately (lateral/heading within the centre tolerance). Contact/surface/command cap use '
                 'green..stop end.'),
        max_sample_interval_s=.05,
        max_sample_interval_basis='current g1d evaluator ego pose coverage maximum_allowed_interval_s (report.evaluation.coverage.ego)',
        bypass_side_rule='ego lateral sign at the first pass sample must be opposite to the target offset sign (+left)',
        trace_header_expected=dict(schema=4,controller_mode='g2_entry_trial',
            execution_config=dict(stationary_outer_boundary=True,wall_reanchor=True,tracked_wall_fit=True,rejoin_enabled=True,
                                  wide_entry=True,prefer_earlier_transition=False),
            follow_config=dict(partial_observation_policy='strict',parallel_hold_trial=False,pass_release_trial=False,
                guard_curve_trial=False,traffic_recovery_trial=False,traffic_path_recovery_trial=False,
                corridor_fallback_enabled=False,short_boundary_prefix_enabled=False,recent_observation_enabled=False,
                boundary_evidence_mode='connected',occupancy_timing_mode='oldest',wheelbase_m=.135,length_m=.2,width_m=.15,
                wall_offset_m=.425,max_steering_rad=0.3710583448410034,max_speed_mps=1.0,lateral_acceleration_mps2=3.0),
            motion_config=dict(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06,lidar_y_m=0.,lidar_yaw_rad=0.,
                gyro_bias_rad_s=0.,wheel_scale=1.,history_s=2.,max_input_gap_s=.05,max_extrapolation_s=.03,scan_timeout_s=.45)),
        trace_header_basis='same launch/defaults as run1 (registered G2 trial); any drift fails instead of being reported only',
        static_target_basis='representative median target pose is valid only for this fixed-target case; actor pose error <= acceptance.harness.actor_static_error_max_m',
        returned_to_center_lateral_m=.05,returned_to_center_heading_rad=.10,returned_to_center_hold_s=1.0,
        returned_basis=('truth-only check after the pass: G2 own centred state uses 25 mm/0.08 rad on observed walls; '
                        'the evaluation allows 50 mm/0.10 rad for wall-vs-centerline geometry differences'),
        actor_contact_messages_with_ego=0,overlap_samples=0,
        max_off_road_wheels=2,wall_wheel_samples=0,
        off_road_basis='official two-wheel grass allowance; any/center/full contact-patch interpretations all reported, '
                       'all must stay <= 2 wheels with zero 3+ wheel events; wheel-centre-only results are not used as contact-patch evidence. '
                       'Patches use MCU target steering (planned geometry), not measured wheel steering',
        command_cap_mps=.35,
        command_cap_scope='/drive and /drive/safe trace samples and every replayed decision output',
        body_gap_policy='minimum body gap is reported only; the old 150 mm G1 line is not a G2 gate (completion-first, ADR 0027)',
        stop_and_shutdown='existing explicit software_stop, dense pose stop evidence, MCU latch, clean shutdown',
        decision_replay='schema4 g2_decisions.jsonl replays every call identically',
        sensor_only='controller/safety/actuation subscriptions sensor-only; /drive only from the controller node and '
                    '/drive/safe only from lidar_safety; actor pose read only by the evaluator')
    config['matrix']={'g2':{'cases':[CASE],'seeds':[0]},
                      'note':'single registered run; no automatic repetition until success'}
    write_new(BASE/CONFIG_NAME,json.dumps(config,ensure_ascii=False,indent=2))
    source=ROOT/'src/arena_bringup/launch/simulation.launch.py'
    original=source.read_text(encoding='utf-8');changed=original
    for old,new in (
        ('    world_directory = gazebo_share / "worlds" / TRACK_DIRECTORIES[track]',
         '    world_directory = Path('+repr(str(WORLD))+')  # registered v2 signal fixture only'),
        ('        camera = profile["camera"]',
         '        camera = profile["camera"]\n        camera["xyz_m"] = [-.075, 0., .30]  # trial fixed rear camera (5-lap trial view)\n'
         '        camera["rpy_rad"] = [0., 0., 0.]'),
        ('        executable = "local_pursuit" if local_mode else',
         '        executable = "g2_entry_trial" if local_mode else'),
        ('    spawn_x, spawn_y, spawn_yaw = slot["x"], slot["y"], slot["yaw_rad"]',
         f'    spawn_x, spawn_y, spawn_yaw = {spawn["x"]!r}, {spawn["y"]!r}, {spawn["yaw_rad"]!r}  # registered G2 centerline start')):
        if changed.count(old)!=1:raise ValueError('launch signature changed: '+old[:60])
        changed=changed.replace(old,new)
    write_new(out/'simulation.launch.py',changed)
    write_new(out/'simulation.diff',''.join(difflib.unified_diff(original.splitlines(True),changed.splitlines(True),
        fromfile='production/simulation.launch.py',tofile='g2_fixture/simulation.launch.py')))
    v2=json.loads((WORLD/'manifest.json').read_text(encoding='utf-8'))
    manifest=dict(scope='G2 explicit left-edge trial launch copy; world is the unchanged v2 signal fixture',case=CASE,
        world_directory=str(WORLD.relative_to(ROOT)),grid_slot_for_actor_anchor=GRID_SLOT,spawn=spawn,
        target=dict(center_gap_m=TARGET_GAP_M,lateral_offset_m=TARGET_LATERAL_M,size_m=[.20,.15,.12],static=True),
        duration_s=DURATION_S,v2_signal_manifest_sources=v2['sources'],
        sources={str(p.relative_to(ROOT)):sha(p) for p in (Path(__file__),source,BASE_CONFIG,WORLD/'world.sdf',
                 WORLD/'scene.json',WORLD/'centerline.csv',WORLD/'manifest.json')})
    write_new(out/'manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2))
    print(json.dumps(dict(spawn=spawn,target=manifest['target']),ensure_ascii=False))


if __name__=='__main__':main()
