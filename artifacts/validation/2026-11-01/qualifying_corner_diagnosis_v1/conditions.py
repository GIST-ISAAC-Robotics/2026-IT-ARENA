"""세 실행의 입력 동일성과 마지막 코너 접근의 명령·전달을 나란히 비교한다."""
import gzip
import hashlib
import json
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
RUNS = {'rise_only': ROOT / 'race_acceleration_v1/run1', 'turn_only': ROOT / 'turn_preview_v1/run4',
        'combined_failed': ROOT / 'qualifying_profile_v1/run1'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def header(folder):
    with (folder / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
        return json.loads(f.readline())['data']


def source_hashes(folder):
    with zipfile.ZipFile(folder / 'source_snapshot.zip') as z:
        return {n: hashlib.sha256(z.read(n)).hexdigest() for n in z.namelist() if not n.endswith('/')}


def approach(name):
    with gzip.open(HERE / f'extract_{name}.json.gz', 'rt', encoding='utf-8') as f:
        rows = [r for r in json.load(f)['trajectory'] if r['light'] == 'green']
    out = {}
    for goal in (1.8, 1.5, 1.2, 1.0):
        hit = next((r for r in rows if r['progress_m'] > 40.5 and r['applied_target_mps'] <= goal), None)
        out[f'first_target_le_{goal}'] = hit and dict(progress=round(hit['progress_m'], 2), t=round(hit['time'], 2),
                                                      body=round(hit['truth_longitudinal_mps'], 2), wheel=round(hit['wheel_surface_speed_mps'], 2))
    window = [r for r in rows if 41.5 <= r['progress_m'] <= 42.6]
    t = window[-1]['time'] - window[0]['time']
    out['window_41p5_42p6'] = dict(duration_s=round(t, 3), body_start=round(window[0]['truth_longitudinal_mps'], 3),
                                   body_end=round(window[-1]['truth_longitudinal_mps'], 3),
                                   body_decel_mps2=round((window[0]['truth_longitudinal_mps'] - window[-1]['truth_longitudinal_mps']) / t, 2),
                                   mean_motor_torque_nm=round(sum(r['motor_torque_nm'] for r in window) / len(window), 4),
                                   mean_target_mps=round(sum(r['applied_target_mps'] for r in window) / len(window), 3))
    return out


result = {'approach': {}, 'identity': {}}
base = None
hashes = {}
for name, folder in RUNS.items():
    h = header(folder)
    report = json.loads((folder / 'report.json').read_bytes())
    item = dict(world_sha=sha(folder / 'runtime_world.sdf'), vehicle_sha=sha(folder / 'runtime_vehicle.sdf'),
                grid_slot=report.get('grid_slot'), speed_bump=report.get('speed_bump_enabled'), speed_profile=report.get('speed_profile'),
                mass=report.get('simulated_total_mass_kg'), lidar_rate=report.get('requested_lidar_rate_hz'),
                control_rate=report.get('requested_control_rate_hz'), follow=h['follow_config'], motion=h['motion_config'])
    hashes[name] = source_hashes(folder)
    result['identity'][name] = {k: v for k, v in item.items() if k not in ('follow', 'motion')}
    if base is None:
        base = item
    else:
        result['identity'][name]['follow_config_diff_from_rise_only'] = {
            k: [base['follow'].get(k), v] for k, v in item['follow'].items() if base['follow'].get(k) != v}
        result['identity'][name]['motion_config_same'] = item['motion'] == base['motion']
    result['approach'][name] = approach(name)
pairs = (('rise_only', 'combined_failed'), ('turn_only', 'combined_failed'))
result['source_files_differ'] = {f'{a}__{b}': sorted(k for k in set(hashes[a]) | set(hashes[b]) if hashes[a].get(k) != hashes[b].get(k)) for a, b in pairs}
with (HERE / 'conditions.json').open('w', encoding='utf-8', newline='\n') as f:
    json.dump(result, f, ensure_ascii=False, indent=1)
print(json.dumps(result['approach'], ensure_ascii=False))
print(json.dumps(result['identity'], ensure_ascii=False)[:1600])
print(result['source_files_differ'])
