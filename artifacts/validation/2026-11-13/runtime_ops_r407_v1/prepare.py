"""정적 YAML/원문/AST 묶음 준비. ROS·후보·serial을 실행하지 않는다."""
import argparse
import ast
import difflib
import hashlib
import importlib.util
import json
from pathlib import Path
import zipfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]


def sha(data):return hashlib.sha256(data).hexdigest()


def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf8',newline='\n')


def read_generated_profile(text):
    """동결 r401 생성물의 2단 mapping+JSON scalar만 수락한다. 일반 YAML parser가 아니다."""
    result={};node=None
    for line in text.splitlines():
        if not line.strip() or line.startswith('#'):continue
        if not line.startswith(' '):
            assert line.endswith(':')
            node=line[:-1];assert node not in result;result[node]={}
        elif line=='  ros__parameters:':
            assert node is not None and not result[node];result[node]['ros__parameters']={}
        else:
            assert line.startswith('    ') and not line.startswith('     '),line
            key,value=line.strip().split(': ',1);parameters=result[node]['ros__parameters']
            assert key not in parameters;parameters[key]=json.loads(value)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--bundle',action='store_true');args=parser.parse_args()
    from registration import derive,validate
    assert not (HERE/'bundle_sources.zip').exists(),'preparation is frozen; new revision required'
    prepared=HERE/'prepared';reference=HERE/'reference'
    prepared.mkdir(exist_ok=True);reference.mkdir(exist_ok=True)
    r401=HERE.parent/'runtime_launch_profile_r401_v1/prepared_v2'
    original={
        'r401_runtime_profile.yaml':r401/'runtime_profile.yaml',
        'r401_bindings.json':r401/'bindings.json',
        'r401_expected_header.json':r401/'expected_header.json',
        'r401_source_evidence.json':r401/'source_evidence.json',
        'r404_RESULT.md':HERE.parent/'runtime_live_r404_v1/RESULT.md',
        'r366_README.md':HERE.parent/'manual_visual_start_r366_v1/README.md',
        'r394_verify_ros_pty_sequence_v2.py':HERE.parent/'runtime_serial_host_r394_v1/revision_motor_v2/verify_ros_pty_sequence_v2.py',
    }
    for name,path in original.items():(reference/name).write_bytes(path.read_bytes())
    source_text=(reference/'r401_runtime_profile.yaml').read_text(encoding='utf8')
    base=read_generated_profile(source_text);selected=derive(base)
    bindings=json.loads((reference/'r401_bindings.json').read_bytes());result=validate(base,selected,bindings)
    selected_text=source_text.replace('race_control_mode: "legacy"','race_control_mode: "b3_candidate"')
    selected_text=selected_text.replace('    development_speed_limit_enabled: false',
        '    development_speed_limit_enabled: true\n    development_speed_limit_mps: 0.3')
    selected_text=selected_text.replace('lidar_safety:\n  ros__parameters:\n',
        'lidar_safety:\n  ros__parameters:\n    race_control_mode: "b3_candidate"\n')
    assert read_generated_profile(selected_text)==selected
    (prepared/'runtime_profile.yaml').write_text(selected_text,encoding='utf8',newline='\n')
    (prepared/'bindings.json').write_bytes((reference/'r401_bindings.json').read_bytes())
    text=(reference/'r394_verify_ros_pty_sequence_v2.py').read_text(encoding='utf8')
    cls=next(n for n in ast.parse(text).body if isinstance(n,ast.ClassDef) and n.name=='FakeMcu')
    old='\n'.join(text.splitlines()[cls.lineno-1:cls.end_lineno])+'\n'
    updated=old.replace('max_speed_mps=1.,max_steering_rad=.45','max_speed_mps=1.5,max_steering_rad=.4578436017036438')
    updated=updated.replace("boot_id='r394-pty-only'","boot_id='r407-pty-only'")
    assert updated!=old and updated.count('class FakeMcu:')==1
    imports='''"""r394 검증된 PTY FakeMcu. r401 속도/조향 상한과 boot 이름만 명시 변경."""
import os
import select
import threading
import time
from arena_vehicle_interface.actuation_wire import Decoder,encode
from arena_vehicle_interface.actuation_contract import ActuationReceiver,ContractConfig
from arena_vehicle_interface.drive_feedback import MODE_MOTOR
from audit_actuation_offer_clock import KEYS as OFFER_KEYS


'''
    (HERE/'fake_mcu.py').write_text(imports+updated,encoding='utf8',newline='\n')
    (HERE/'fake_mcu.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),updated.splitlines(True),
        fromfile='r394/FakeMcu',tofile='r407/FakeMcu')),encoding='utf8',newline='\n')
    (HERE/'profile.patch').write_text(''.join(difflib.unified_diff(
        json.dumps(base,ensure_ascii=False,indent=2).splitlines(True),
        json.dumps(selected,ensure_ascii=False,indent=2).splitlines(True),
        fromfile='r401/runtime_profile.semantic.json',tofile='r407/runtime_profile.semantic.json')),
        encoding='utf8',newline='\n')
    # These are the prior PTY assumptions, not measured physical timing/calibration.
    policy=dict(schema='runtime_host_clock_policy_v1',receiver_lease=dict(source_ttl_ns=100_000_000,
        max_receipt_age_ns=100_000_000,max_rate_error_ppm=1000,policy_note='Declared PTY test assumption; not hardware verified'),
        same_host=dict(source_ttl_ns=100_000_000,max_lookback_ns=100_000_000,max_sample_gap_ns=50_000_000,
        max_read_width_ns=1_000_000,ros_timestamp_error_ns=1000,host_timestamp_error_ns=0,max_rate_error_ppm=1000,
        assume_no_unsampled_clock_step=True,policy_note='Same-host PTY test assumptions; unsampled jumps unverified'),
        max_lease_us=100_000,request_timeout_ns=200_000_000)
    write(prepared/'clock_policy_pty.json',policy)
    write(prepared/'synthetic_saved_race.json',dict(v=1,vehicle_id='r407_pty_only',race_id='r407-synthetic-prior-race',
        session='r407-synthetic-prior-session',race_epoch=0,side='left',side_marker=None,
        start_source='synthetic_fixture_not_visual_observation',saved_wall_time=0.,synthetic=True,
        scope='PTY only previous-race fixture. Not a real race, image recognition, or physical motion proof.'))
    ledger={name:dict(source=path.relative_to(ROOT).as_posix(),sha256=sha(path.read_bytes())) for name,path in original.items()}
    write(HERE/'source_inputs.json',dict(reference=ledger,registration=result,executed=False,
        fake_mcu_changes=['boot label','declared final profile speed/steering ceilings; all DRIVE targets remain zero'],
        common_source_edits=0))
    if not args.bundle:
        print(json.dumps(dict(prepared=True,executed=False,reference_files=len(original))));return
    spec=importlib.util.spec_from_file_location('r407_static_closure',ROOT/'scripts/python_dependency_closure.py')
    closure_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(closure_module)
    names=('run_once.py','run_ops.py','registration.py','fake_mcu.py','preflight.py')
    overlay={'r407/'+name:HERE/name for name in names}
    class Sources:
        def __contains__(self,name):return name in overlay or (ROOT/name).is_file()
        def __getitem__(self,name):return overlay.get(name,ROOT/name).read_bytes()
    entries=list(overlay)+['scripts/run_runtime_actuation_host.py']
    closure=closure_module.collect(entries,roots=closure_module.PACKAGE_ROOTS+closure_module.SCRIPT_ROOTS+('r407',),members=Sources())
    members={name:overlay.get(name,ROOT/name).read_bytes() for name in closure['files']}
    for folder in (prepared,reference):
        for path in folder.iterdir():members['r407/'+path.relative_to(HERE).as_posix()]=path.read_bytes()
    for name,data in members.items():
        if name.endswith('.py'):ast.parse(data,filename=name)
    manifest=dict(schema='r407_ops_bundle_v1',files={name:sha(data) for name,data in members.items()},
        closure=closure,reference_inputs=ledger,common_source_edits=0,executed=False)
    write(HERE/'bundle_manifest.json',manifest)
    with zipfile.ZipFile(HERE/'bundle_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name,data in sorted(members.items()):z.writestr(name,data)
        z.write(HERE/'bundle_manifest.json','bundle_manifest.json')
    print(json.dumps(dict(frozen_files=len(members),bundle_sha256=sha((HERE/'bundle_sources.zip').read_bytes()),executed=False)))


if __name__=='__main__':main()
