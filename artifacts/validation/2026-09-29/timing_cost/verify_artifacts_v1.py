"""회수한 결과와 분석 소스를 감사·보존한다. 시험 자료를 수정하지 않는다."""
import hashlib
import json
from pathlib import Path, PurePosixPath
import tarfile
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def main():
    archive = ROOT/'build/timing_cost_results_v1.tar.gz'
    expected = 'd3fb086ebd0cee1dcaf3d4786a6ddaf276e2b57ec92616b68bee006fef51577c'
    assert sha(archive.read_bytes()) == expected
    files = {}
    with tarfile.open(archive) as tar:
        members = tar.getmembers()
        for member in members:
            relative = PurePosixPath(member.name)
            assert not relative.is_absolute() and '..' not in relative.parts
            assert member.isdir() or member.isfile()
            if member.isfile():
                assert member.name not in files
                assert not member.name.endswith('.mcap') and '/sensor_recording/' not in member.name
                payload = tar.extractfile(member).read()
                target = HERE/'jetson_results'/member.name
                assert target.is_file() and sha(target.read_bytes()) == sha(payload)
                files[member.name] = dict(size=len(payload), sha256=sha(payload))
    comparison = json.loads((HERE/'comparison_v1.json').read_text())
    source = comparison['runs'][0]['source_sha256']
    assert all(sha((ROOT/name).read_bytes()) == digest for name, digest in source.items())
    previous = ROOT/'artifacts/validation/2026-09-29/imu_long_capture/jetson_results/long_capture_v3_a/replay/source_snapshot.zip'
    with zipfile.ZipFile(previous) as old:
        changed = [name for name in old.namelist() if sha(old.read(name)) != sha((ROOT/name).read_bytes())]
    analyses = [ROOT/'scripts'/name for name in ('summarize_timing_cost.py', 'summarize_dds_trials.py',
        'summarize_jetson_trials.py', 'summarize_controller_performance.py', 'analyze_imu_delivery.py',
        'analyze_wheel_delivery.py', 'replay_timing.py', 'replay_observer.py', 'replay_transport.py',
        'replay_imu_probe.py')]
    analyses += [ROOT/'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py',
        HERE/'analyze_stop_checkpoints_v1.py', Path(__file__).resolve(),
        ROOT/'tests/test_timing_cost_summary.py']
    source_hashes = {p.relative_to(ROOT).as_posix(): sha(p.read_bytes()) for p in analyses}
    with zipfile.ZipFile(HERE/'analysis_source_v1.zip', 'x', zipfile.ZIP_DEFLATED) as saved:
        for p in analyses:
            saved.write(p, p.relative_to(ROOT).as_posix())
    result = dict(archive_sha256=expected, archive_size=archive.stat().st_size,
        members=len(members), files=files, raw_recording_included=False,
        runtime_source_files=len(source), runtime_source_matches_current=True,
        changed_previous_runtime_sources=changed, analysis_sources=source_hashes,
        analysis_archive_sha256=sha((HERE/'analysis_source_v1.zip').read_bytes()))
    with (HERE/'artifact_audit_v1.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(dict(members=len(members), files=len(files), runtime_source_files=len(source),
        changed_previous_runtime_sources=changed, analysis_source_files=len(analyses))))


if __name__ == '__main__':
    main()
