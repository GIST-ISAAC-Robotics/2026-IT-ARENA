"""최종 소스 선택 뒤 기존 collector/설치 생성기를 조합한다. 전송·설치·장치 I/O 없음."""
import argparse
import importlib.util
import json
import math
from pathlib import Path, PurePosixPath
import re
import zipfile

import prepare_hardware_policy as policy
from arena_autonomy.vehicle_geometry import equivalent_steering_limit, float32_limit_not_above

ROOT=policy.ROOT
HERE=Path(__file__).resolve().parent
COLLECTOR=ROOT/'artifacts/validation/2026-11-14/runtime_handover_r491_v1/stage/scripts/prepare_runtime_handover.py'
COLLECTOR_SHA='1d44f8f1cffc218538347406805451599731ec83514ca039abbb1ce5b45fa226'
R500=ROOT/'artifacts/validation/2026-11-14/runtime_handover_r491_v1/final_v1'
R500_SHA=dict(archive='73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965',
              manifest='ee450fef0d971d2b94e75544420de5e13d6e9e694b4d2a12bdf6f848581098a5')
sha=policy.limits.digest
save=policy.save_json
load=policy.installed.read_json
SECRET=re.compile(rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|\bsk-proj-[A-Za-z0-9_-]{30,}|\bgh[pousr]_[A-Za-z0-9]{30,}|\bAKIA[A-Z0-9]{16}\b')
STEERING_SELECTION='handover/steering_selection.json'
STEERING_GUIDE='handover/STEERING_SELECTION.md'


def check_steering_selection(value,files):
    """선택한 두 공개 파일과 기존 CAD/ready 계약만 확인한다. board 값을 만들지 않는다."""
    if (value['schema']!='runtime_source_steering_selection_v1'
            or value['source_only'] is not True or value['installed'] is not False
            or value['hardware_angle_measured'] is not False):
        raise ValueError('steering selection must remain source-only with CAD wheel angles')
    for key,prefix in (('steering_profile','config/hardware/'),('vehicle_config','src/arena_description/config/')):
        item=value[key];name=item['path']
        if not name.startswith(prefix) or name not in files or sha(files[name])!=item['sha256']:
            raise ValueError('selected steering source missing or SHA mismatch: '+name)
    selected={value[key]['path']:files[value[key]['path']] for key in ('steering_profile','vehicle_config')}
    check_public(selected)
    zero=json.loads(selected[value['steering_profile']['path']])
    drive=policy.yaml.safe_load(selected[value['vehicle_config']['path']])['vehicle']['drivetrain']
    limit=value['command_limit_rad']
    if (type(limit) not in (int,float) or not math.isfinite(limit) or limit<=0
            or float32_limit_not_above(limit)!=limit
            or float32_limit_not_above(equivalent_steering_limit(drive))!=limit
            or drive['max_steering_angle_rad']!=value['simulation_joint_limit_rad']):
        raise ValueError('selected vehicle/host central steering limit mismatch')
    policy.installed.cpp_zero_profile(zero,assembly_id=value['assembly_id'],command_limit=limit,
        wheelbase=drive['wheelbase_m'],track=drive['track_width_m'],wheel_limit=math.pi/6)
    return value


def steering_files(value):
    if value is None:return {}
    files={}
    for key in ('steering_profile','vehicle_config'):
        name=value[key]['path'];path=PurePosixPath(name)
        if path.is_absolute() or '..' in path.parts or '\\' in name or ':' in name:
            raise ValueError('selected steering source must be a repository-relative path')
        source=ROOT/name
        if source.is_symlink():raise ValueError('selected steering source cannot be a symlink')
        files[name]=source.read_bytes()
    check_steering_selection(value,files)
    files[STEERING_SELECTION]=(json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode('utf-8')
    files[STEERING_GUIDE]=steering_guide(value).encode('utf-8')
    files['handover/steering_launch_selection.py']=(HERE/'steering_launch_selection.py').read_bytes()
    return files


def check_installed_steering(value,files,board_path,zero_path):
    drive=policy.yaml.safe_load(files[value['vehicle_config']['path']])['vehicle']['drivetrain']
    expected=dict(command_limit=value['command_limit_rad'],wheelbase=drive['wheelbase_m'],
                  track=drive['track_width_m'],wheel_limit=math.pi/6)
    board=load(board_path)
    if (sha(Path(zero_path).read_bytes())!=value['steering_profile']['sha256']
            or any(board['servo'][key]!=number for key,number in expected.items())):
        raise ValueError('explicit installed inputs differ from selected steering profile/limit/geometry')


def steering_guide(value):
    return f'''# 선택한 실측 서보 입력 — 설치 전 단계

영점/운영 count 프로필: `{value['steering_profile']['path']}`
시뮬레이션 vehicle 입력: `{value['vehicle_config']['path']}`
중앙 명령 상한 L: `{value['command_limit_rad']}` rad. vehicle YAML의 내륜 joint 상한과 구분한다.
두 파일의 SHA는 `steering_selection.json`에 기록했다. wheel각은 CAD 환산이며 실측값이 아니다.
기존 preview 실행을 선택할 때는 `run_hardware_sensor_preview.py --vehicle-config {value['vehicle_config']['path']}`를 명시한다.
이 옵션은 기하 입력이며 실제 host/MCU 서보 설정을 대신하지 않는다.

검토한 실제 board 사본의 `servo.command_limit`만 L로 선택하고 나머지 실제 board/PI/encoder 값을 유지한다.
아직 실제 board가 없으면 생성하지 않는다. 임의 motor/PI/포트 값으로 빈칸을 채우지 않는다.

```sh
python scripts/runtime_installed_profile.py prepare --board-profile "$REVIEWED_BOARD" --steering-profile {value['steering_profile']['path']} --assembly-id {value['assembly_id']} --output "$FRESH_INSTALLED"
python handover/prepare_hardware_policy.py prepare --base-params "$REVIEWED_PARAMS" --hardware-inputs "$REAL_INPUTS" --installed-profile "$FRESH_INSTALLED" --output "$FRESH_SELECTION"
python scripts/runtime_installed_limits.py host-arguments --selection "$FRESH_SELECTION/selected" --installed-profile "$FRESH_INSTALLED"
```

planner/guard에는 같은 `$FRESH_SELECTION/selected/runtime_profile.yaml`, host에는 출력된 arguments를 사용한다.
native firmware 빌드에는 같은 `$FRESH_INSTALLED`를 기존 `build_runtime_drive.sh --installed-profile`로 전달한다.
인자 연결은 `python handover/steering_launch_selection.py --steering-selection handover/steering_selection.json --installed-profile "$FRESH_INSTALLED" --runtime-selection "$FRESH_SELECTION/selected"`로 확인한다.
guard 상태 기본은 `full`이다. r547과 같은 compact 선택은 위 명령에 `--guard-detail compact_v1 --guard-result-path "$FRESH_RUN_ABSOLUTE_PATH"`를 명시하며 새 절대 경로의 `guard_status_contexts.jsonl`을 함께 회수한다.
반환된 `guard_with_status` 인자는 params-file 뒤에 선택 status 매개변수를 전달한다. 기존 r547 replay profile 전체를 실제 운영 YAML에 복사하지 않는다. `--runtime-params "$FINAL_RUNTIME_YAML"`은 양쪽 실제 상한이 선택 L과 같은지 소스 전용으로도 확인한다.
수동 panel 상한은 기존 `min(0.2, 실행 상한)`을 유지한다. START/재개는 실행 중인 설정을 사용하며 새 프로필을 자동 선택하지 않는다.
ZIP 교체나 vehicle YAML 선택만으로 MCU/host에 적용되지 않는다. r500 설치본은 rollback 기준으로 보존한다.
이 source 묶음은 전송·설치·flash·실물 출력 완료를 의미하지 않는다.
'''


def collector():
    if sha(COLLECTOR.read_bytes())!=COLLECTOR_SHA:
        raise ValueError('preserved r491 collector changed')
    spec=importlib.util.spec_from_file_location('preserved_r491_collector',COLLECTOR)
    tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
    return tool


def source_files(tool,steering_selection=None):
    files=tool.collect(ROOT)
    policy.check_policy(policy.policy_overlay())
    files.update({
        'handover/prepare_hardware_policy.py':(HERE/'prepare_hardware_policy.py').read_bytes(),
        'handover/policy_overlay.yaml':policy.yaml.safe_dump(policy.policy_overlay(),sort_keys=False).encode('utf-8'),
        'handover/required_inputs.json':(json.dumps(policy.required_inputs(),indent=2)+'\n').encode('utf-8')})
    files.update(steering_files(steering_selection))
    return files


def check_public(files):
    excluded={'build','install','log','__pycache__','.git','.venv','private','.private','outputs','output','artifacts','.ssh','node_modules'}
    suffixes={'.so','.dll','.exe','.elf','.bin','.a','.o','.pyd','.pyc','.pem','.key','.p12','.pfx','.bag','.db3','.mcap'}
    for name,raw in files.items():
        path=PurePosixPath(name)
        if (path.is_absolute() or '..' in path.parts or '\\' in name or ':' in name
                or excluded.intersection(path.parts) or path.suffix.lower() in suffixes
                or path.name in {'.env','.netrc','id_rsa','id_ed25519','credentials.json'}):
            raise ValueError('excluded source path: '+name)
        if raw.startswith((b'\x7fELF',b'MZ',b'\xcf\xfa\xed\xfe',b'\xfe\xed\xfa\xcf')) or SECRET.search(raw):
            raise ValueError('binary/key signature in source path: '+name)


def baseline():
    if (sha((R500/'runtime_handover_sources.zip').read_bytes())!=R500_SHA['archive']
            or sha((R500/'manifest.json').read_bytes())!=R500_SHA['manifest']):
        raise ValueError('preserved r500 source baseline changed')
    return load(R500/'manifest.json')


def delta(old,new):
    return [dict(path=name,before_sha256=old.get(name),after_sha256=new.get(name))
            for name in sorted(old.keys()|new.keys()) if old.get(name)!=new.get(name)]


def prepare(output,selection,*,base_params=None,hardware_inputs=None,board_profile=None,steering_profile=None,
            steering_selection=None):
    if not isinstance(selection,str) or not selection.strip():
        raise ValueError('root final source selection label required')
    supplied=(base_params,hardware_inputs,board_profile,steering_profile)
    if any(value is not None for value in supplied) and not all(value is not None for value in supplied):
        raise ValueError('explicit base/hardware/board/mounted steering inputs must be supplied together')
    steering=load(steering_selection) if steering_selection is not None else None
    tool=collector();old=baseline();files=source_files(tool,steering);check_public(files)
    if steering is not None and all(value is not None for value in supplied):
        check_installed_steering(steering,files,board_profile,steering_profile)
    manifest=tool.inventory(files)
    if manifest['required_missing']:
        raise ValueError('required sources missing: '+', '.join(manifest['required_missing']))
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    policy.draft(output/'policy_draft')
    if all(value is not None for value in supplied):
        # 기존 generator가 neutral-only/잘못된 assembly/실장 범위 불일치를 거절한다.
        assembly=load(hardware_inputs)['assembly_id']
        installed=output/'private/installed_profile'
        policy.installed.prepare(board_profile,steering_profile,assembly,installed)
        policy.prepare(Path(base_params),Path(hardware_inputs),installed,output/'private/runtime_selection')
    archive=output/'runtime_handover_sources.zip'
    tool.write_bundle(files,archive)
    if tool.inventory(source_files(tool,steering))['files']!=manifest['files']:
        raise ValueError('source changed during preparation; candidate is not validated')
    manifest['archive_sha256']=sha(archive.read_bytes());save(output/'manifest.json',manifest)
    save(output/'delta_from_r500.json',dict(baseline_sha256=R500_SHA,files=delta(old['files'],manifest['files'])))
    save(output/'handover_receipt.json',dict(final_source_selection=selection,collector_sha256=COLLECTOR_SHA,
        baseline_sha256=R500_SHA,hardware_selection_prepared=all(value is not None for value in supplied),
        steering_selection_prepared=steering is not None,
        installed=False,install_pending=True,hardware_ready=False,measured_inputs_verified=False,
        firmware_written=False,device_io=False,source_only=True,
        servo_probe_firmware_rebuild_complete=False))
    result=validate(output,against_current=True)
    save(output/'verification_receipt.json',result)
    return result


def validate(output,*,against_current=False):
    output=Path(output);tool=collector();baseline()
    manifest=load(output/'manifest.json');receipt=load(output/'handover_receipt.json')
    archive=output/'runtime_handover_sources.zip'
    if sha(archive.read_bytes())!=manifest['archive_sha256']:
        raise ValueError('source ZIP hash changed')
    with zipfile.ZipFile(archive) as zipped:
        names=zipped.namelist()
        if (len(names)!=len(set(names)) or set(names)!=set(manifest['files'])|{'source_manifest.json'} or zipped.testzip()):
            raise ValueError('ZIP member/CRC mismatch')
        files={name:zipped.read(name) for name in manifest['files']}
        check_public(files)
        expected=tool.inventory(files)
        if (expected!={k:v for k,v in manifest.items() if k!='archive_sha256'}
                or json.loads(zipped.read('source_manifest.json'))!=expected or expected['required_missing']):
            raise ValueError('source manifest or file SHA mismatch')
    if (receipt['collector_sha256']!=COLLECTOR_SHA or receipt['baseline_sha256']!=R500_SHA
            or any(receipt[key] is not False for key in ('installed','hardware_ready','measured_inputs_verified','firmware_written','device_io'))):
        raise ValueError('handover receipt cannot claim installation or measured readiness')
    steering=json.loads(files[STEERING_SELECTION]) if STEERING_SELECTION in files else None
    if receipt.get('steering_selection_prepared',False)!=(steering is not None):
        raise ValueError('steering selection receipt/member mismatch')
    if steering is not None:
        check_steering_selection(steering,files)
        # 이전 선택본은 당시 manifest가 고정한 안내 bytes를 검증한다. 새 안내와 비교해 소급 무효화하지 않는다.
        if not files.get(STEERING_GUIDE):
            raise ValueError('steering handover guide missing')
        if (b'handover/steering_launch_selection.py' in files[STEERING_GUIDE]
                and 'handover/steering_launch_selection.py' not in files):
            raise ValueError('steering launch selection helper missing')
    if against_current and expected['files']!=tool.inventory(source_files(tool,steering))['files']:
        raise ValueError('selected source differs from current workspace')
    if receipt['hardware_selection_prepared']:
        selected=output/'private/runtime_selection/selected'
        policy.limits.verify(selected,output/'private/installed_profile')
        policy.check_policy(policy.limits.profile_values((selected/'runtime_profile.yaml').read_bytes()))
        if steering is not None:
            installed=output/'private/installed_profile'
            check_installed_steering(steering,files,installed/'runtime_board.json',installed/'steering_zero.json')
    return dict(ok=True,source_file_count=len(files),archive_sha256=manifest['archive_sha256'],
        hardware_selection_prepared=receipt['hardware_selection_prepared'],
        steering_selection_prepared=steering is not None,install_pending=True,installed=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    draft=commands.add_parser('draft');draft.add_argument('--output',type=Path,required=True)
    make=commands.add_parser('prepare');make.add_argument('--output',type=Path,required=True)
    make.add_argument('--final-source-selection',required=True)
    make.add_argument('--steering-selection',type=Path)
    for flag in ('base-params','hardware-inputs','board-profile','steering-profile'):
        make.add_argument('--'+flag,type=Path)
    check=commands.add_parser('validate');check.add_argument('--output',type=Path,required=True)
    check.add_argument('--against-current',action='store_true')
    selected=commands.add_parser('selection-check');selected.add_argument('--steering-selection',type=Path,required=True)
    args=parser.parse_args()
    try:
        if args.command=='draft':
            policy.draft(args.output);result=dict(policy_draft_only=True,archive_created=False,installed=False)
        elif args.command=='prepare':
            result=prepare(args.output,args.final_source_selection,**{k:getattr(args,k) for k in
                ('base_params','hardware_inputs','board_profile','steering_profile','steering_selection')})
        elif args.command=='selection-check':
            value=load(args.steering_selection);steering_files(value)
            result=dict(steering_selection_valid=True,command_limit_rad=value['command_limit_rad'],
                        archive_created=False,installed=False,device_io=False)
        else:result=validate(args.output,against_current=args.against_current)
    except (ValueError,OSError,TypeError,KeyError,policy.yaml.YAMLError,zipfile.BadZipFile) as error:
        print(json.dumps(dict(error=str(error),installed=False,device_io=False)));return 2
    print(json.dumps(result));return 0


if __name__=='__main__':
    raise SystemExit(main())
