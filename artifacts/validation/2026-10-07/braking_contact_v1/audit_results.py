"""현재 런타임/보존 소스/물리 동일성을 독립 대조. 실패 판정은 변경하지 않는다."""
import hashlib
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def normalized_model(path):
    root = ET.parse(path).getroot()
    count = 0
    for link in root.findall('model/link'):
        for sensor in list(link.findall("sensor[@name='eval_wheel_contact']")):
            link.remove(sensor)
            count += 1
    return ET.canonicalize(ET.tostring(root, encoding='unicode'), strip_text=True), count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--include-capture-v2', action='store_true')
    parser.add_argument('--output-name', default='final_audit.json')
    args = parser.parse_args()
    if Path(args.output_name).name != args.output_name:
        raise ValueError('output must be a filename in the evidence folder')
    original = ROOT/'artifacts/validation/2026-10-07/no_pillars_v1/after_r0_stopped_s0_repeat/runtime_vehicle.sdf'
    original_model, _ = normalized_model(original)
    runs = {}
    names = ['r0_s0', 'fixed_official', 'fixed_official_v2', 'fixed_flat_v2']
    if args.include_capture_v2:
        names.append('r0_s0_capture_v2')
    for name in names:
        folder = HERE/name
        report = json.loads((folder/'report.json').read_text())
        with zipfile.ZipFile(folder/'source_snapshot.zip') as bundle:
            checks = {path: hashlib.sha256(bundle.read(path)).hexdigest() == value
                      for path, value in report['source_sha256'].items()}
        model, sensors = normalized_model(folder/'runtime_vehicle.sdf')
        raw_checks = {key: sha(folder/f'{key}.jsonl') == report[key]['sha256']
                      for key in ('wheel_contacts', 'braking_diagnostics')}
        runs[name] = {'source_zip_matches': all(checks.values()), 'raw_hashes_match': all(raw_checks.values()),
                      'same_vehicle_except_four_evaluation_sensors': model == original_model and sensors == 4,
                      'report_sha256': sha(folder/'report.json'), 'source_zip_sha256': sha(folder/'source_snapshot.zip'),
                      'preserved_verdict': {key: report.get(key) for key in ('passed', 'run_valid', 'input_valid',
                          'stopped', 'shutdown_clean', 'source_unchanged', 'error')},
                      'runtime_vehicle_sha256': sha(folder/'runtime_vehicle.sdf'),
                      'runtime_world_sha256': sha(folder/'runtime_world.sdf')}
    unchanged = json.loads((ROOT/'config/tests/g1b_r0_sha256.json').read_text())['sha256']
    checks = {path: sha(ROOT/path) == expected for path, expected in unchanged.items() if not path.endswith('simulation.launch.py')}
    result = {'runs': runs, 'controller_sensor_physics_contract_unchanged': checks,
              'scope': 'evidence integrity and unchanged vehicle; not override of run validity or behavior failures'}
    result['passed'] = all(checks.values()) and all(
        all(row[key] for key in ('source_zip_matches', 'raw_hashes_match', 'same_vehicle_except_four_evaluation_sensors')) for row in runs.values())
    with (HERE/args.output_name).open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
