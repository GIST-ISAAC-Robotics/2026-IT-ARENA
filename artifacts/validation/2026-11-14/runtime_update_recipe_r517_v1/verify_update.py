"""r500 방식의 source/install SHA와 변경 진입점만 확인. 최종 동결/설치 후 별도 실행."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import subprocess
import sys


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--baseline-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root, base = args.root.resolve(), args.baseline_root.resolve()
    if root == base or args.output.exists():
        parser.error('distinct update root and fresh result path required')
    manifest = json.loads((root/'source_manifest.json').read_text())
    baseline = json.loads((base/'source_manifest.json').read_text())
    result = dict(source_mismatch=[], installed_mismatch=[], changed_imports={}, cli_help={},
                  source_count=len(manifest['files']), installed_values=manifest['installed_values'],
                  device_io=False, nodes_started=False, firmware_written=False,
                  installed_values_selected=False, controller_runtime_verified=False)
    changed = [name for name, sha in manifest['files'].items() if baseline['files'].get(name) != sha]
    result['changed_from_r500'] = changed
    result['removed_from_r500'] = sorted(baseline['files'].keys()-manifest['files'].keys())
    for name, expected in manifest['files'].items():
        if digest(root/name) != expected:
            result['source_mismatch'].append(name)
        parts = name.split('/')
        if len(parts) < 4 or parts[0] != 'src' or parts[1] not in ('arena_autonomy', 'arena_vehicle_interface') or parts[2] != parts[1] or not name.endswith('.py'):
            continue
        installed = root/'install'/parts[1]/'lib/python3.10/site-packages'/Path(*parts[2:])
        if digest(installed) != expected:
            result['installed_mismatch'].append(name)
        if name in changed:
            module_name = '.'.join(parts[2:])[:-3]
            module = importlib.import_module(module_name)
            result['changed_imports'][module_name] = str(Path(module.__file__).resolve())
            if Path(module.__file__).resolve() != installed.resolve():
                result['installed_mismatch'].append('import:'+name)
    native = importlib.import_module('arena_autonomy._numeric_kernels')
    result['native'] = dict(path=str(Path(native.__file__).resolve()), sha256=digest(Path(native.__file__)),
                            abi=native.ABI_VERSION)
    native_ok = (Path(native.__file__).resolve().is_relative_to(root/'install')
        and result['native']['sha256'] == '5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff'
        and native.ABI_VERSION == 1)
    for script, extra in (('steering_zero_profile.py', ['goal']), ('mounted_servo_setup.py', [])):
        call = subprocess.run([sys.executable, '-B', str(root/'scripts'/script), *extra, '--help'],
                              capture_output=True, text=True, timeout=20)
        result['cli_help'][script] = dict(exit=call.returncode, stderr=call.stderr[-1000:])
        if script == 'steering_zero_profile.py':
            result['cli_help'][script]['selected_limit_exposed'] = '--command-limit-rad' in call.stdout
    import serial
    result['serial_path'] = str(Path(serial.__file__).resolve())
    dependency_hashes = lambda folder: {p.relative_to(folder).as_posix(): digest(p)
        for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    result['deps_match_r500'] = dependency_hashes(root/'deps') == dependency_hashes(base/'deps')
    result['c1_sha256'] = digest(root/'bin/stream_c1_native_preview')
    result['ok'] = (not result['source_mismatch'] and not result['installed_mismatch'] and native_ok
        and Path(serial.__file__).resolve().is_relative_to(root/'deps')
        and result['deps_match_r500']
        and result['c1_sha256'] == 'd8f7d2a02ef47786022206e2a0278c6e7edc2d3ddb40bed2254706b590748e2d'
        and all(v['exit'] == 0 for v in result['cli_help'].values())
        and result['cli_help']['steering_zero_profile.py']['selected_limit_exposed']
        and all(v is None for v in manifest['installed_values'].values()))
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result, indent=2))
    return 0 if result['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
