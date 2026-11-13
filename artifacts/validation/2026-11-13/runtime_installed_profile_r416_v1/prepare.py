"""r416의 필요한 고정 소스만 수집한다. WSL/빌드/장치 실행은 하지 않는다."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
STAGED = A/'staged'


def main():
    archive = A/'sources.zip'
    if archive.exists(): raise FileExistsError('frozen archive exists; do not overwrite')
    names = set()
    for pattern in ('src/arena_vehicle_interface/portable/*.hpp',
        'src/arena_vehicle_interface/firmware_runtime_preview/main/*',
        'src/arena_vehicle_interface/firmware_runtime_drive/main/*'):
        names.update(p.relative_to(ROOT).as_posix() for p in ROOT.glob(pattern) if p.is_file())
    names.update('src/arena_vehicle_interface/'+app+'/'+name
        for app in ('firmware_runtime_preview', 'firmware_runtime_drive')
        for name in ('CMakeLists.txt', 'sdkconfig.defaults'))
    names.update(['scripts/build_runtime_preview.sh', 'scripts/build_runtime_drive.sh',
        'scripts/runtime_installed_profile.py', 'scripts/steering_zero_profile.py',
        'tests/test_runtime_installed_profile.py', 'tests/test_steering_calibration.py'])
    names.update('src/arena_vehicle_interface/arena_vehicle_interface/'+name
        for name in ('__init__.py', 'steering_calibration.py', 'steering_cad_table.py'))
    raw = {}; originals = {}; changes = []; patch = []
    for name in sorted(names):
        chosen = STAGED/name if (STAGED/name).exists() else ROOT/name
        raw[name] = chosen.read_bytes()
        if name.endswith('.py'): ast.parse(raw[name].decode('utf-8-sig'), filename=name)
        if chosen.is_relative_to(STAGED):
            old = (ROOT/name).read_bytes() if (ROOT/name).exists() else b''
            originals[name] = hashlib.sha256(old).hexdigest() if old else None
            if old: raw['_r416/original/'+name] = old
            changes.append(name)
            patch.extend(difflib.unified_diff(old.decode('utf-8-sig').replace('\r\n','\n').splitlines(True),
                raw[name].decode('utf-8-sig').replace('\r\n','\n').splitlines(True),
                fromfile='a/'+name, tofile='b/'+name))
    raw['_r416/verify.py'] = (A/'verify.py').read_bytes()
    ast.parse(raw['_r416/verify.py'].decode('utf-8'), filename='verify.py')
    manifest = dict(schema='runtime_installed_profile_r416_sources_v1',
        files={name: hashlib.sha256(value).hexdigest() for name, value in sorted(raw.items())},
        staged_files=changes, original_common_sha256=originals, executed=False)
    manifest_bytes = (json.dumps(manifest, indent=2)+'\n').encode('utf-8')
    (A/'source_manifest.json').write_bytes(manifest_bytes)
    (A/'staged.patch').write_text(''.join(patch), encoding='utf-8', newline='\n')
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as bundle:
        for name, value in sorted(raw.items()): bundle.writestr(name, value)
        bundle.writestr('source_manifest.json', manifest_bytes)
    receipt = dict(source_files=len(raw), archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
        manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
        syntax_parsed=True, control_or_device_executed=False)
    (A/'preparation.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(receipt))


if __name__ == '__main__': main()
