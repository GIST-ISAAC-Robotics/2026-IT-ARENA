"""최종 소스 선택 뒤 기존 collector/설치 생성기를 조합한다. 전송·설치·장치 I/O 없음."""
import argparse
import importlib.util
import json
from pathlib import Path, PurePosixPath
import re
import zipfile

import prepare_hardware_policy as policy

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


def collector():
    if sha(COLLECTOR.read_bytes())!=COLLECTOR_SHA:
        raise ValueError('preserved r491 collector changed')
    spec=importlib.util.spec_from_file_location('preserved_r491_collector',COLLECTOR)
    tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
    return tool


def source_files(tool):
    files=tool.collect(ROOT)
    policy.check_policy(policy.policy_overlay())
    files.update({
        'handover/prepare_hardware_policy.py':(HERE/'prepare_hardware_policy.py').read_bytes(),
        'handover/policy_overlay.yaml':policy.yaml.safe_dump(policy.policy_overlay(),sort_keys=False).encode('utf-8'),
        'handover/required_inputs.json':(json.dumps(policy.required_inputs(),indent=2)+'\n').encode('utf-8')})
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


def prepare(output,selection,*,base_params=None,hardware_inputs=None,board_profile=None,steering_profile=None):
    if not isinstance(selection,str) or not selection.strip():
        raise ValueError('root final source selection label required')
    supplied=(base_params,hardware_inputs,board_profile,steering_profile)
    if any(value is not None for value in supplied) and not all(value is not None for value in supplied):
        raise ValueError('explicit base/hardware/board/mounted steering inputs must be supplied together')
    tool=collector();old=baseline();files=source_files(tool);check_public(files)
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
    if tool.inventory(source_files(tool))['files']!=manifest['files']:
        raise ValueError('source changed during preparation; candidate is not validated')
    manifest['archive_sha256']=sha(archive.read_bytes());save(output/'manifest.json',manifest)
    save(output/'delta_from_r500.json',dict(baseline_sha256=R500_SHA,files=delta(old['files'],manifest['files'])))
    save(output/'handover_receipt.json',dict(final_source_selection=selection,collector_sha256=COLLECTOR_SHA,
        baseline_sha256=R500_SHA,hardware_selection_prepared=all(value is not None for value in supplied),
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
    if against_current and expected['files']!=tool.inventory(source_files(tool))['files']:
        raise ValueError('selected source differs from current workspace')
    if receipt['hardware_selection_prepared']:
        selected=output/'private/runtime_selection/selected'
        policy.limits.verify(selected,output/'private/installed_profile')
        policy.check_policy(policy.limits.profile_values((selected/'runtime_profile.yaml').read_bytes()))
    return dict(ok=True,source_file_count=len(files),archive_sha256=manifest['archive_sha256'],
        hardware_selection_prepared=receipt['hardware_selection_prepared'],install_pending=True,installed=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    draft=commands.add_parser('draft');draft.add_argument('--output',type=Path,required=True)
    make=commands.add_parser('prepare');make.add_argument('--output',type=Path,required=True)
    make.add_argument('--final-source-selection',required=True)
    for flag in ('base-params','hardware-inputs','board-profile','steering-profile'):
        make.add_argument('--'+flag,type=Path)
    check=commands.add_parser('validate');check.add_argument('--output',type=Path,required=True)
    check.add_argument('--against-current',action='store_true')
    args=parser.parse_args()
    try:
        if args.command=='draft':
            policy.draft(args.output);result=dict(policy_draft_only=True,archive_created=False,installed=False)
        elif args.command=='prepare':
            result=prepare(args.output,args.final_source_selection,**{k:getattr(args,k) for k in
                ('base_params','hardware_inputs','board_profile','steering_profile')})
        else:result=validate(args.output,against_current=args.against_current)
    except (ValueError,OSError,TypeError,KeyError,policy.yaml.YAMLError,zipfile.BadZipFile) as error:
        print(json.dumps(dict(error=str(error),installed=False,device_io=False)));return 2
    print(json.dumps(result));return 0


if __name__=='__main__':
    raise SystemExit(main())
