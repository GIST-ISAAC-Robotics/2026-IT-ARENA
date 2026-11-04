"""수정된 후방 분류만 동일 scenario에 연결하기 전 소스 보존."""
import hashlib
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def main():
    old = json.loads((HERE/'registered.json').read_bytes())
    scenario = 'artifacts/validation/2026-11-03/side_departure_runtime_v1/scenario.json'
    if hashlib.sha256((ROOT/scenario).read_bytes()).hexdigest() != old['source_sha256'][scenario]:
        raise ValueError('scenario changed')
    files = set(old['source_sha256'])
    files.update(p.relative_to(ROOT).as_posix() for p in HERE.glob('*') if p.suffix in ('.py', '.sh', '.md'))
    files.add('config/tests/side_departure_rear_r2_sha256.json')
    registered = dict(route=old['route'], source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(files)})
    with (HERE/'registered2.json').open('x', encoding='utf-8') as f:
        json.dump(registered, f, ensure_ascii=False, indent=2)
    with zipfile.ZipFile(HERE/'scene_source2.zip', 'x', compression=zipfile.ZIP_DEFLATED) as z:
        for p in sorted(files):
            z.write(ROOT/p, p)
    print(json.dumps(dict(source_count=len(files), scenario_unchanged=True, route=old['route'])))


if __name__ == '__main__':
    main()
