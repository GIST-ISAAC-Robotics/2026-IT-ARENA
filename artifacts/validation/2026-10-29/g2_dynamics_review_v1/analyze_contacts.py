"""G2 진단의 바퀴별 하중/접촉 쌍. 누락은 0하중으로 채우지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from analyze_wheel_contacts import contact_rows, WHEELS


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def strip_contact_sensors(path):
    root = ET.parse(path).getroot()
    found = []
    for link in root.findall('model/link'):
        for sensor in link.findall('sensor'):
            if sensor.get('name') == 'eval_wheel_contact':
                found.append(link.get('name'))
                link.remove(sensor)
    return ET.canonicalize(ET.tostring(root, encoding='unicode')), found


def world_geometry_equal(first,second):
    """실행별 임시 폴더 이름만 정규화. OBJ 내용은 별도 해시 일치가 필요하다."""
    roots=[ET.parse(folder/'runtime_world.sdf').getroot() for folder in (first,second)]
    meshes=[]
    for root,folder in zip(roots,(first,second)):
        hashes={}
        for uri in root.iter('uri'):
            name=Path(uri.text or '').name
            if name.startswith('batched_') and name.endswith('.obj'):
                hashes[name]=sha(folder/name)
                uri.text=name
        meshes.append(hashes)
    return (meshes[0]==meshes[1] and
            ET.canonicalize(ET.tostring(roots[0],encoding='unicode'))==
            ET.canonicalize(ET.tostring(roots[1],encoding='unicode')))


def analyze(folder, baseline):
    report = json.loads((folder/'report.json').read_text(encoding='utf-8'))
    raw = folder/'wheel_contacts.jsonl'
    assert sha(raw) == report['wheel_contacts']['sha256'], 'raw contact hash mismatch'
    trace = json.loads((folder/'trace.json').read_text(encoding='utf-8'))
    moving = [r for r in trace if r.get('elapsed_since_green_s') is not None]
    green = moving[0]['t']-moving[0]['elapsed_since_green_s']
    windows = {'straight': (green+.5, green+1.5),
               'transition': (green+4.8, green+5.8),
               'late': (green+6., green+11.9)}
    rows = {name: {w: [] for w in WHEELS} for name in windows}
    last = {}
    duplicates = 0
    conflicts = 0
    with raw.open(encoding='utf-8') as stream:
        for line in stream:
            for row in contact_rows(json.loads(line)):
                key = (row['wheel'], row['t'])
                prev = last.get(row['wheel'])
                if prev and prev['t'] == row['t']:
                    if prev == row:
                        duplicates += 1
                        continue
                    conflicts += 1
                    raise ValueError('conflicting contact record at same time')
                last[row['wheel']] = row
                for name, (lo, hi) in windows.items():
                    if lo <= row['t'] <= hi:
                        rows[name][row['wheel']].append(row)
    summary = {}
    for name, wheels in rows.items():
        lo, hi = windows[name]
        summary[name] = {'window_sim_s': [lo, hi], 'wheels': {}}
        for wheel, contacts in wheels.items():
            contacts.sort(key=lambda r: r['t'])
            loads = [r['force_world_xyz_n'][2] for r in contacts if r['force_available']]
            others = Counter(c for r in contacts for c in r['other_collisions'])
            summary[name]['wheels'][wheel] = dict(
                contact_samples=len(contacts), valid_force_samples=len(loads),
                fz_min_p05_median_p95_max_n=np.percentile(loads, [0, 5, 50, 95, 100]).tolist() if loads else None,
                max_observed_gap_s=max((b['t']-a['t'] for a,b in zip(contacts, contacts[1:])), default=None),
                first_s=contacts[0]['t'] if contacts else None,
                last_s=contacts[-1]['t'] if contacts else None,
                other_collisions=dict(others),
                note='No row means unobserved contact, not measured zero load')
        dynamics = [r for r in moving if lo <= r['t'] <= hi]
        fields = ('truth_longitudinal_mps', 'wheel_surface_speed_mps', 'left_speed_rad_s', 'right_speed_rad_s', 'motor_torque_nm')
        summary[name]['dynamics_medians'] = {
            field: float(np.median([r['braking_diagnostics'][field] for r in dynamics])) if dynamics else None for field in fields}
    before, old_sensors = strip_contact_sensors(baseline/'runtime_vehicle.sdf')
    after, new_sensors = strip_contact_sensors(folder/'runtime_vehicle.sdf')
    return dict(run=str(folder), input_sha256={p.name: sha(p) for p in (raw, folder/'report.json', folder/'trace.json', folder/'runtime_vehicle.sdf')},
                same_vehicle_except_contact_sensors=before==after,
                old_contact_sensors=old_sensors, new_contact_sensors=new_sensors,
                same_runtime_world=sha(folder/'runtime_world.sdf')==sha(baseline/'runtime_world.sdf'),
                same_world_after_verified_mesh_path_normalization=world_geometry_equal(folder,baseline),
                duplicate_rows_ignored=duplicates, conflicting_rows=conflicts, windows=summary,
                limit='Recorded contacts only; contact force and geometry are simulator diagnostics, not hardware measurements.')


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args()
    result=analyze(args.run,args.baseline)
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps({k:result[k] for k in ('same_vehicle_except_contact_sensors','new_contact_sensors','same_runtime_world','duplicate_rows_ignored')}))
    for name,window in result['windows'].items():
        print(name,window['dynamics_medians'])
        print({w:dict(samples=r['contact_samples'],fz=r['fz_min_p05_median_p95_max_n']) for w,r in window['wheels'].items()})
