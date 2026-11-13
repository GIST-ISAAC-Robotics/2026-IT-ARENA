"""r457 원본을 바꾸지 않는 Windows 전용 r464 기록 추출."""
from pathlib import Path
import hashlib
import json
import zipfile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
SOURCE = HERE.parent / 'solo_full_lap_r457_v1'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def compact(value):
    if isinstance(value, dict):
        return {k: compact(v) for k, v in value.items()
                if k not in ('path_points', 'path', 'rays', 'ranges', 'scan', 'history',
                             'occupied_endpoints_xy_m', 'input', 'plan', 'source_indices')}
    if isinstance(value, list):
        return [compact(v) for v in value] if len(value) < 20 else {'count': len(value)}
    return value

def main():
    manifest = {'source_archive': str(SOURCE / 'sources.zip'),
                'archive_sha256': sha(SOURCE / 'sources.zip'), 'files': {}}
    assert manifest['archive_sha256'] == '1e966fd9e27eae75dba8b11c56a5018a98c7971569daca39f8c248ec2da24b21'
    with zipfile.ZipFile(SOURCE / 'sources.zip') as archive:
        for name in archive.namelist():
            if ((name.startswith('src/arena_autonomy/') and name.endswith('.py')) or
                name in ('src/arena_vehicle_interface/arena_vehicle_interface/__init__.py',
                         'src/arena_vehicle_interface/arena_vehicle_interface/queued_log.py',
                         'tests/test_local_follow.py', 'tests/test_boundary_bridge.py',
                         'tests/test_traffic_boundary_refresh.py', 'tests/test_junction_priority.py',
                         'tests/test_same_scan_history.py')):
                target = HERE / 'original' / name
                target.parent.mkdir(parents=True, exist_ok=True)
                data = archive.read(name)
                if target.exists():
                    assert target.read_bytes() == data
                else:
                    target.write_bytes(data)
                manifest['files'][name] = hashlib.sha256(data).hexdigest()
    rows, summary, pairs = [], [], {}
    with (SOURCE / 'run1/follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            event = json.loads(line)
            data = event.get('data', {})
            seq = data.get('sequence')
            if event['kind'] in ('runtime_plan_submitted', 'runtime_plan_completed') and seq is not None:
                if 627 <= seq <= 645:
                    rows.append(event)
                    pairs.setdefault(seq, {})[event['kind']] = {'line': line_number, 'event': event}
                    if event['kind'] == 'runtime_plan_completed':
                        summary.append({'line': line_number, 'trace_seq': event['seq'],
                            'plan_sequence': seq, 'output': compact(data.get('output'))})
    (HERE / 'original_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    (HERE / 'saved_plan_window.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows), encoding='utf-8')
    (HERE / 'plan_diagnostics.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps({'source_sha256': manifest['archive_sha256'], 'copied_python_files': len(manifest['files']),
                      'saved_events': len(rows), 'plan_pairs': len(pairs)}))
    for row in summary:
        out = row['output']
        normal = out[2].get('normal', {})
        keys = ['reason', 'reference_stamp_s', 'source_first_s', 'fresh_boundary_accepted',
                'boundary_age_s', 'boundary_from_memory', 'path_wall', 'boundary_selection',
                'boundary_candidate_reason', 'boundary_candidate_ahead_m',
                'junction_wall_switch_rejected', 'expiry_route_switch_rejected',
                'current_corner_support', 'recent_boundary_confirmation',
                'target_speed_before_hold_mps', 'swept_clearance_m', 'partial_observation']
        print(json.dumps({'plan': row['plan_sequence'], 'line': row['line'], 'speed': out[0],
            'steering': out[1], 'normal': {k: normal[k] for k in keys if k in normal}}))

if __name__ == '__main__':
    main()
