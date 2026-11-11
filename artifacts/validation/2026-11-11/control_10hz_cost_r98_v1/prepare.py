"""r91 run4의 완료 기록과 당시 소스만 격리 재생 묶음으로 보존한다."""
from pathlib import Path
import hashlib
import io
import json
import math
import sys
import tarfile
import time
import zipfile

ROOT = Path(__file__).resolve().parents[4]
A = Path(__file__).resolve().parent
D = ROOT / 'artifacts/validation/2026-11-11/control_10hz_r91_v1/two_actor_run4'
sha = lambda data: hashlib.sha256(data).hexdigest()


def main():
    # 판정의 성공 여부와 비용 측정 가능 여부는 다르다. 종료/입력의 무결성이 필요하다.
    assessment = json.loads((D / 'combined_assessment.json').read_bytes())
    assert assessment['gates']['control_dt_0p1_verified']
    assert assessment['gates']['snapshot_unchanged']
    trace = D / 'run1/follow_decisions.jsonl'
    ncall = 0
    footer = None
    with trace.open(encoding='utf8') as stream:
        for line in stream:
            event = json.loads(line)
            assert footer is None
            if event['kind'] == 'call':
                assert math.isclose(event['data']['input']['dt_s'], .1, abs_tol=1e-12)
                ncall += 1
            elif event['kind'] == 'footer':
                footer = event['data']
    assert ncall > 0 and footer is not None
    assert all(footer.get(k) == 0 for k in ('capture_errors', 'failed', 'dropped', 'pending'))
    assert footer.get('worker_alive') is False

    observer = A / 'measure_stages.py'
    original = ROOT / 'artifacts/validation/2026-11-11/jetson_completed_traffic_r88_v1/measure_stages.py'
    text = original.read_text(encoding='utf8')
    text = text.replace("scope='r87 completed native traffic trace, current frozen r86 policy; instrumented original replay_call; no ROS or actuation deadline claim'",
                        "scope='r91 run4 actual 10Hz trace; immutable replay, stage CPU cost only; no ROS or actuation deadline claim'")
    text = text.replace('over_20ms=sum(v>20 for v in values)',
                        'over_20ms=sum(v>20 for v in values),over_100ms=sum(v>100 for v in values)')
    compile(text, str(observer), 'exec')
    assert not observer.exists()
    observer.write_text(text, encoding='utf8')
    sys.path.insert(0, str(ROOT / 'scripts'))
    from prepare_jetson_bundle import follow_profile_tools
    files = list(follow_profile_tools())
    files += [trace.relative_to(ROOT).as_posix(), observer.relative_to(ROOT).as_posix()]
    assert len(files) == len(set(files))
    snapshot = D / 'extra_sources.zip'
    with zipfile.ZipFile(snapshot) as z:
        common = sorted(set(z.namelist()) & set(files))
        changed = [n for n in common if sha(z.read(n)) != sha((ROOT / n).read_bytes())]
    assert not changed, changed
    archive = ROOT / 'build/jetson_10hz_r98_20261111.tar.gz'
    local = ROOT / 'build/r98_pc_frozen'
    assert not archive.exists() and not local.exists() and not (A / 'bundle_manifest.json').exists()
    inventory = {}
    local.mkdir()
    with tarfile.open(archive, 'x:gz', compresslevel=1) as out:
        for name in files:
            data = (ROOT / name).read_bytes()
            item = tarfile.TarInfo(name)
            item.size, item.mtime = len(data), int(time.time())
            out.addfile(item, io.BytesIO(data))
            dest = local / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(data)
            inventory[name] = dict(size=len(data), sha256=sha(data))
    manifest = dict(files=inventory, archive_sha256=sha(archive.read_bytes()),
                    archive_size=archive.stat().st_size,
                    trace=trace.relative_to(ROOT).as_posix(), observer=observer.relative_to(ROOT).as_posix(),
                    calls=ncall, driving_passed=assessment['passed'],
                    scope='r91 actual 10Hz trace and matching source; stage cost, no ROS/actuation',
                    driving_snapshot_sha256=sha(snapshot.read_bytes()),
                    snapshot_common_files=len(common), snapshot_changed=changed)
    (A / 'bundle_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k: v for k, v in manifest.items() if k != 'files'}))


if __name__ == '__main__':
    main()
