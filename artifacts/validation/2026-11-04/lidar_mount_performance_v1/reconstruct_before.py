"""이번 시험 전용 수정 직전 바이트를 역적용으로 재구성하고 기존 기록과 대조한다.

출력: runtime_before.zip(수정 전 7개 파일). 이미 있으면 바이트 일치만 확인한다.
"""
import hashlib
import io
import json
from pathlib import Path
import sys
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def remove_block(text, start, end):
    a = text.index(start)
    b = text.index(end, a)
    return text[:a] + text[b:]


def drop_lines(text, nl, predicates):
    lines, out, skip = text.split(nl), [], 0
    for line in lines:
        if skip:
            skip -= 1
            continue
        hit = next((n for p, n in predicates if p(line)), None)
        if hit is None:
            out.append(line)
        else:
            skip = hit
    return nl.join(out)


def before(path, data):
    nl = '\r\n' if b'\r\n' in data else '\n'
    text = data.decode('utf-8')
    if path.endswith('local_scene.py'):
        text = remove_block(text, 'def blind_spec(value):', 'def rear_masked(')
        new = text[text.index('def rear_masked('):text.index('def scene_from_scan(')]
        old = nl.join(["def rear_masked(angle_rad, rear_half_angle_deg=30.):",
            "    angle = math.remainder(_finite(angle_rad, 'angle_rad'), math.tau)",
            "    half = _finite(rear_half_angle_deg, 'rear_half_angle_deg')",
            "    if not 0 <= half < 180:",
            "        raise ValueError('invalid rear mask angle')",
            "    # 센서 축 기준. ±150° 경계의 표본은 현행 mask_scan과 같이 관측한다.",
            "    return abs(angle) > math.radians(180 - half) + 1e-12", "", "", ""])
        text = text.replace(new, old)
    elif path.endswith('local_path.py'):
        text = drop_lines(text, nl, [
            (lambda l: 'from arena_autonomy.local_scene import blind_spec' in l, 1),
            (lambda l: '# 시험 전용 장착 구조 각도. 거리 반환이 있어도' in l, 3)])
        text = text.replace('math.radians(180 - rear)] = np.nan', 'math.radians(180 - rear_half_angle_deg)] = np.nan')
    elif path.endswith(('local_pursuit.py', 'lidar_safety.py')):
        text = drop_lines(text, nl, [(lambda l: '# 시험 전용 장착 구조 각도(JSON)' in l, 3)])
    elif path.endswith('simulation.launch.py'):
        text = remove_block(text, 'MOUNT_TRIAL_KEYS = (', 'def _local_sensor_profile(')
        text = drop_lines(text, nl, [
            (lambda l: l.strip() == 'blind_sectors_json = ""', 0),
            (lambda l: 'model_xml, blind_sectors_json = _mount_trial(' in l, 2),
            (lambda l: l.strip() == 'if blind_sectors_json:', 1),
            (lambda l: 'lidar_blind_sectors_json' in l, 0)])
    elif path.endswith('validate_basic_autonomy.py'):
        text = drop_lines(text, nl, [
            (lambda l: "parser.add_argument('--local-sensor-config'" in l, 1),
            (lambda l: l.strip() == 'if args.local_sensor_config is not None:', 3)])
    elif path.endswith('validate_g1_traffic.py'):
        text = drop_lines(text, nl, [
            (lambda l: "parser.add_argument('--local-sensor-config'" in l, 1),
            (lambda l: l.strip() == 'if args.local_sensor_config is not None:', 1)])
    assert all(k not in text for k in ('blind_spec', 'blind_parameter', 'blind_sectors', 'mount_trial', 'local-sensor-config', 'local_sensor_config:=')), path
    return text.encode('utf-8')


FILES = ('src/arena_autonomy/arena_autonomy/local_scene.py', 'src/arena_autonomy/arena_autonomy/local_path.py',
         'src/arena_autonomy/arena_autonomy/local_pursuit.py', 'src/arena_autonomy/arena_autonomy/lidar_safety.py',
         'src/arena_bringup/launch/simulation.launch.py', 'scripts/validate_basic_autonomy.py', 'scripts/validate_g1_traffic.py')
REFERENCES = ('artifacts/validation/2026-11-03/right_reapproach_v3/scene_source.zip',)


def main():
    rows, data = {}, {}
    known = {}
    for name in REFERENCES:
        with zipfile.ZipFile(ROOT/name) as archive:
            for item in archive.namelist():
                known.setdefault(item, {})[name] = sha(archive.read(item))
    contracts = {p.name: p.read_text(encoding='utf-8') for p in (ROOT/'config/tests').glob('*.json')}
    for path in FILES:
        current = (ROOT/path).read_bytes()
        old = before(path, current)
        data[path] = old
        digest = sha(old)
        rows[path] = dict(before=digest, after=sha(current),
                          snapshot_match={k: v == digest for k, v in known.get(path, {}).items()},
                          contracts=sorted(n for n, t in contracts.items() if digest in t)[-3:])
    print(json.dumps(rows, ensure_ascii=False, indent=1))
    target = HERE/'runtime_before.zip'
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            info = zipfile.ZipInfo(path, date_time=(2026, 11, 4, 0, 0, 0))
            archive.writestr(info, data[path], compress_type=zipfile.ZIP_DEFLATED)
    if target.exists():
        with zipfile.ZipFile(target) as archive:
            assert all(archive.read(p) == data[p] for p in FILES), 'before snapshot differs'
    elif '--write' in sys.argv:
        target.write_bytes(buffer.getvalue())
    if not all(r['snapshot_match'].get(REFERENCES[0], True) for r in rows.values()):
        raise SystemExit('reconstructed bytes differ from latest run snapshot')


if __name__ == '__main__':
    main()
