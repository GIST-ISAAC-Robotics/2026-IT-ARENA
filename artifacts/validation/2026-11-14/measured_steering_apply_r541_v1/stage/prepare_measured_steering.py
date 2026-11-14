"""보존한 장착 측정+현재 운영 선택을 기존 ready 프로필로 연결. 장치 I/O 없음."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import tarfile

ROOT=next(parent for parent in Path(__file__).resolve().parents
          if (parent/'scripts/runtime_installed_profile.py').is_file())
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'src/arena_autonomy')]
import runtime_installed_profile as installed
from arena_vehicle_interface.steering_calibration import (
    CadMapping,read_profile,complete_mount_profile,check_profile,covers,cad_servo_goal,cpp_zero_profile,save_new_profile)
from arena_autonomy.vehicle_geometry import float32_limit_not_above

MEASURED=ROOT/'build/mounted_servo_setup_20261114_v1/manual-capture.MMQBmq-verified'
BENCH=ROOT/'artifacts/validation/2026-11-12/st3215_servo_bench_v1/summary.json'
DEPLOYED=ROOT/'build/mounted_servo_setup_20261114_v1/manual-three-point-v1.tar.gz'
DEPLOY_LOG=ROOT/'build/mounted_servo_setup_20261114_v1/manual-three-point-jetson.log'
GEOMETRY=dict(wheelbase=.128,track=.1239,wheel_limit=math.pi/6)
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def selected_profile(minimum=2600,maximum=2980):
    zero=read_profile(MEASURED/'straight.json')
    manual=installed.read_json(MEASURED/'summary.json')
    powered=installed.read_json(MEASURED/'powered-sweep-summary.json')
    identity=installed.read_json(BENCH)['runs']['motion_observe_5cycles']['identity']
    if any(identity[key]!=value for key,value in dict(id=1,mode=0,resolution=1,angle_min=0,angle_max=4095).items()):
        raise ValueError('preserved ID/mode/resolution evidence mismatch')
    with tarfile.open(DEPLOYED) as archive:
        deployed=archive.extractfile('scripts/mounted_servo_setup.py').read()
    deployed_sha=hashlib.sha256(deployed).hexdigest()
    if deployed_sha not in DEPLOY_LOG.read_text(encoding='utf-8'):
        raise ValueError('deployed mounted tool SHA differs from preserved receipt')
    if not manual['completed'] or manual['torque_after']!=0 or manual['issues'] or manual['crosses_count_wrap']:
        raise ValueError('manual evidence incomplete')
    points=manual['manual_points']
    if (manual['assembly_id']!=zero['assembly_id'] or points['straight']['shaft_count']!=zero['zero_count']
            or any(points[key]['wheel_angle_deg'] is not None for key in points)):
        raise ValueError('assembly/zero/angle evidence changed; review new evidence')
    direction=manual['count_direction_per_cad_positive']
    left,right=points['left_max']['shaft_count'],points['right_max']['shaft_count']
    if (left-zero['zero_count'])*direction>=0 or (right-zero['zero_count'])*direction<=0:
        raise ValueError('recorded count direction contradicts left/right points')
    stages=powered['stages'];targets=[row['target'] for row in stages]
    if not powered['completed'] or powered['torque_after']!=0 or not stages:
        raise ValueError('powered evidence incomplete')
    for row in stages:
        if (row['torque_after']!=0 or (row['position_before_release']-row['start'])*(row['target']-row['start'])<=0
                or abs(row['position_before_release']-row['target'])>powered['tolerance_counts']
                or abs(row['released']-row['target'])>powered['tolerance_counts']):
            raise ValueError('powered direction/settlement evidence mismatch')
    if not min(targets)<=minimum<zero['zero_count']<maximum<=max(targets):
        raise ValueError('selected window exceeds powered target coverage')
    if not min(left,right)<minimum<maximum<max(left,right):
        raise ValueError('selected window must remain inside manual endpoints')
    value=complete_mount_profile(zero,direction=direction,minimum=minimum,maximum=maximum,confirmed=True)
    value['selection_basis']=dict(
        selected_on='2026-11-14',selection='user_requested_measured_range_adoption',
        manual_operating_limits_confirmed_at_capture=manual['operating_limits_confirmed'],
        operating_limits_selected_now=True,manual_count_interval=[min(left,right),max(left,right)],
        powered_target_interval=[min(targets),max(targets)],
        powered_observed_interval=[min(row['position_before_release'] for row in stages),max(row['position_before_release'] for row in stages)],
        wheel_angles_measured=False,geometry_source='preserved Fusion CAD; not measured wheel angles',
        flag_meanings=dict(straight_confirmed='mounted manual straight 2811 counts',
            direction_confirmed='manual left/right count signs and powered movement direction',
            endpoints_confirmed='current user-selected inner powered target window; not mechanical stops',
            position_mode_confirmed='11/12 mode=0 readback; 11/14 completed deployed bounds() check',
            scale_confirmed='resolution=1 and 0..4095 single-turn protocol convention; not optical angle calibration'),
        deployed_tool_sha256=deployed_sha,
        source_sha256={str(path.relative_to(ROOT)).replace('\\','/'):sha(path) for path in
            (MEASURED/'straight.json',MEASURED/'summary.json',MEASURED/'powered-sweep-summary.json',BENCH,DEPLOYED,DEPLOY_LOG)})
    check_profile(value,ready=True,assembly_id=zero['assembly_id'])
    return value


def derive(profile):
    check_profile(profile,ready=True)
    mapping=CadMapping.default();zero=profile['zero_count'];direction=profile['count_direction_per_cad_positive']
    edges=[]
    for count in (profile['safe_min_count'],profile['safe_max_count']):
        relative=(count-zero)*math.tau/(direction*4096)
        goal=mapping.relative(relative)
        if goal.clamped:raise ValueError('selected endpoint outside retained CAD table')
        edges.append(dict(count=count,shaft_delta_deg=(count-zero)*360/4096,
            cad_servo_relative_rad=relative,central_equivalent_rad=goal.equivalent_rad,
            left_wheel_cad_rad=goal.left_rad,right_wheel_cad_rad=goal.right_rad))
    low,high=0.,min(mapping.command_limit(**GEOMETRY),*[abs(e['central_equivalent_rad']) for e in edges])
    raw_limit=high
    for _ in range(55):
        mid=(low+high)/2
        relative=[mapping.central(sign*mid,**GEOMETRY).servo_rad for sign in (-1,1)]
        if covers(profile,min(relative),max(relative)):low=mid
        else:high=mid
    limit=float32_limit_not_above(low)
    goals=[cad_servo_goal(profile,sign*limit,command_limit=limit,**GEOMETRY) for sign in (-1,0,1)]
    native_header,digest=cpp_zero_profile(profile,assembly_id=profile['assembly_id'],command_limit=limit,**GEOMETRY)
    joint_limit=max(max(abs(g.left_rad),abs(g.right_rad)) for g in
                    [mapping.central(sign*raw_limit,**GEOMETRY) for sign in (-1,1)])
    negative=abs(min(e['central_equivalent_rad'] for e in edges))
    return dict(geometry=GEOMETRY,endpoint_cad_estimates=edges,command_limit_rad=limit,
        command_limit_deg=math.degrees(limit),raw_symmetric_limit_rad=raw_limit,
        sim_joint_inner_limit_rad=joint_limit,
        goals_right_zero_left=goals,
        right_unused_count=goals[0]['position']-profile['safe_min_count'],
        right_unused_central_deg=math.degrees(negative-limit),
        right_unused_fraction=(negative-limit)/negative,
        canonical_profile_sha256=digest,hardware_angle_measured=False,installed=False),native_header


def prepare(output,minimum=2600,maximum=2980):
    profile=selected_profile(minimum,maximum)
    report,header=derive(profile)
    output.mkdir(parents=True,exist_ok=False)
    save_new_profile(output/'steering_zero.selected.json',profile)
    (output/'mounted_steering_zero.hpp').write_text(header,encoding='ascii')
    write(output/'derived_limits.json',report)
    write(output/'board_steering_patch.json',dict(servo=dict(command_limit=report['command_limit_rad']),
        geometry_required=GEOMETRY,assembly_id=profile['assembly_id'],
        note='Merge only servo.command_limit into explicitly reviewed board; motor/PI/speed/IO fields unchanged'))
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--minimum-count',type=int,default=2600)
    parser.add_argument('--maximum-count',type=int,default=2980)
    args=parser.parse_args()
    try:report=prepare(args.output,args.minimum_count,args.maximum_count)
    except (ValueError,OSError,KeyError) as error:
        print(json.dumps(dict(error=str(error),device_io=False)));return 2
    print(json.dumps({key:report[key] for key in ('command_limit_rad','right_unused_count','hardware_angle_measured','installed')}))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
