"""승인 후 지정 담당자가 실행할 새 Jetson 전용 루트 설치/단일 계측. 자동 접속 없음."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import zipfile


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): value.update(block)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--sha256', required=True)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--build-native', action='store_true', help='새 독립 루트 안의 확장 빌드를 명시 허용')
    args = parser.parse_args()
    assert sys.platform == 'linux' and os.uname().machine == 'aarch64', 'Jetson arm64 only'
    archive, root = args.archive.resolve(), args.root.resolve()
    assert digest(archive) == args.sha256
    assert root.is_relative_to(Path.home()) and root != Path.home() and not root.exists()
    assert root.name.startswith('it-arena-r380-'), 'new r380 root only'
    root.mkdir()
    with zipfile.ZipFile(archive) as zipped:
        for name in zipped.namelist():
            assert not Path(name).is_absolute() and '..' not in Path(name).parts and '\\' not in name
        zipped.extractall(root)
    work = root/'artifacts/validation/2026-11-13/current_cost_r380_v1'
    manifest = json.loads((work/'source_manifest.json').read_bytes())
    def changed():
        return [n for n, h in manifest['files'].items() if not (root/n).is_file() or digest(root/n) != h]
    assert not changed()
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(str(root/p) for p in
        ('src/arena_autonomy', 'src/arena_vehicle_interface')), PYTHONDONTWRITEBYTECODE='1')
    steps = []; failure = None; began = time.monotonic()
    def execute(label, command, cwd, timeout):
        with (work/(label+'.log')).open('x') as log:
            child = subprocess.Popen(command, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            record = dict(label=label, command=command, pid=child.pid, timeout_s=timeout, forced=False)
            steps.append(record)
            try:
                record['returncode'] = child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                record['forced'] = True
                os.killpg(child.pid, signal.SIGTERM)
                try: record['returncode'] = child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL); record['returncode'] = child.wait(timeout=5)
                raise
            if record['returncode'] != 0: raise RuntimeError((label, record['returncode']))
    try:
        if args.build_native:
            execute('native_build', ['/usr/bin/python3', '-B', 'setup.py', 'build_ext', '--inplace'],
                    root/'src/arena_autonomy', 120)
        execute('import_preflight', ['/usr/bin/python3', '-B', str(work/'preflight.py'),
            '--backend', 'native_v1', '--output', str(work/'import_preflight.json')], root, 60)
        execute('jetson_run', ['/usr/bin/python3', '-B', str(work/'run.py'),
            '--output', str(work/'jetson'), '--require-machine', 'aarch64'], root, 900)
    except Exception as error:
        failure = repr(error)
    finally:
        result = dict(passed=failure is None, error=failure, steps=steps, wall_s=time.monotonic()-began,
            changed_sources=changed(), source_count=len(manifest['files']),
            source_archive_sha256=args.sha256, devices_opened=False,
            power_settings_changed=False, firmware_changed=False,
            process_policy='each owned child waited; timeout uses own process group only')
        (work/'execution.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
        destination = archive.with_name(archive.stem+'-results.zip')
        assert not destination.exists(), 'preserve previous results'
        with zipfile.ZipFile(destination, 'x', zipfile.ZIP_DEFLATED) as zipped:
            for path in [*work.glob('*.log'), work/'execution.json', work/'import_preflight.json',
                         *work.glob('jetson/*.json'), *work.glob('jetson/*.jsonl')]:
                if path.is_file(): zipped.write(path, path.relative_to(work).as_posix())
        print(json.dumps(result), flush=True)
    if failure or result['changed_sources']: raise SystemExit(1)


if __name__ == '__main__': main()
