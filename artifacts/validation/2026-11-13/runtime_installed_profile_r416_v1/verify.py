#!/usr/bin/env python3
"""r416 격리 사본의 순수/native 검사와 합성 프로필 IDF 빌드 1회. flash/장치 실행 없음."""
import argparse
import hashlib
import importlib.util
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import time
import traceback
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--st-source', type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve(strict=True); output = args.output.resolve()
    if output.exists() or output.with_suffix('.zip').exists() or args.build_dir.exists():
        raise ValueError('fresh output/zip/build paths required; no rerun over failure')
    output.mkdir(parents=True)
    manifest = json.loads((root/'source_manifest.json').read_text())
    report = dict(schema='runtime_installed_profile_r416_result_v1', passed=False,
                  hardware_executed=False, firmware_flashed=False, synthetic_profile_only=True, steps=[])
    def hashes():
        result = {}
        for name in manifest['files']:
            try: result[name] = hashlib.sha256((root/name).read_bytes()).hexdigest()
            except OSError as exc: result[name] = {'unavailable': str(exc)}
        return result
    def step(name, argv, timeout):
        start = time.monotonic()
        with (output/(name+'.log')).open('wb') as log:
            value = subprocess.Popen(argv, cwd=root, env={**os.environ, 'PYTHONDONTWRITEBYTECODE':'1'},
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            timed_out = False
            try: value.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                timed_out = True
                os.killpg(value.pid, signal.SIGTERM)
                try: value.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(value.pid, signal.SIGKILL); value.wait(timeout=10)
        report['steps'].append(dict(name=name, argv=[str(x) for x in argv], exit=value.returncode,
                                    timed_out=timed_out, seconds=time.monotonic()-start))
        if value.returncode or timed_out: raise RuntimeError(name+' failed; original log preserved')
    try:
        before = hashes(); (output/'source_before.json').write_text(json.dumps(before, indent=2)+'\n')
        if before != manifest['files']: raise ValueError('frozen sources mismatch')
        step('tests', [sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
            '--basetemp', str(output/'pytest_tmp'), 'tests/test_runtime_installed_profile.py',
            'tests/test_steering_calibration.py::SteeringCalibrationTests::test_cpp_export_requires_same_verified_mount_and_preserves_offsets',
            'tests/test_steering_calibration.py::SteeringCalibrationTests::test_cpp_export_cli_is_offline_and_refuses_overwrite'], 240)
        spec = importlib.util.spec_from_file_location('r416_synthetic_test_fixture', root/'tests/test_runtime_installed_profile.py')
        fixture = importlib.util.module_from_spec(spec); spec.loader.exec_module(fixture)
        folder = output/'synthetic_inputs_ONLY_NOT_FOR_INSTALL'; folder.mkdir()
        board, _, bp, zp = fixture.synthetic_inputs(folder)
        profile = output/'synthetic_profile_ONLY_NOT_FOR_INSTALL'
        fixture.subject.prepare(bp, zp, board['assembly_id'], profile)
        step('idf_build', ['bash', str(root/'scripts/build_runtime_drive.sh'), str(args.build_dir),
            str(args.st_source), str(output/'build'), '--installed-profile', str(profile)], 1200)
        report['passed'] = True
    except Exception as exc:
        report['error'] = str(exc)
        (output/'traceback.txt').write_text(traceback.format_exc())
    finally:
        after = hashes(); (output/'source_after.json').write_text(json.dumps(after, indent=2)+'\n')
        report['sources_unchanged'] = after == manifest['files']
        report['passed'] = report['passed'] and report['sources_unchanged']
        (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        with zipfile.ZipFile(output.with_suffix('.zip'), 'x', zipfile.ZIP_DEFLATED) as bundle:
            for path in sorted(output.rglob('*')):
                if path.is_file(): bundle.write(path, path.relative_to(output))
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
