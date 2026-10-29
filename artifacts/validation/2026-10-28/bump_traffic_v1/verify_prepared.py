"""실행 전 메타데이터와 실제 고정 인수·소스ZIP을 대조한다. 주행 판정 아님."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import zipfile


def verify(base, name):
    root = base.parents[3]
    metadata = json.loads((base/(name+'_prepared.json')).read_text(encoding='utf-8'))
    hashes = metadata['source_sha256']
    sha = lambda data: hashlib.sha256(data).hexdigest()
    with zipfile.ZipFile(base/(name+'_sources.zip')) as archive:
        members = archive.namelist()
        assert len(members) == len(set(members)), 'duplicate snapshot member'
        assert all(sha(archive.read(path)) == value == sha((root/path).read_bytes())
                   for path, value in hashes.items()), 'source mismatch'
    config = json.loads((base/'traffic_config.json').read_text(encoding='utf-8'))
    case = next(case for case in config['cases'] if case['name'] == metadata['route']['case'])
    assert case['duration_s'] == metadata['route']['duration_s'] == 155
    assert config['acceptance'][case['name']]['minimum_laps'] == metadata['route']['minimum_laps'] == 1
    assert metadata['route']['config'] == str(base/'traffic_config.json')
    assert config['launch']['speed_bump'] and metadata['launch_arguments']['speed_bump'] == 'true'
    assert metadata['controller_parameters']['g1_traffic_recovery_trial']
    assert metadata['controller_parameters']['g1_traffic_path_recovery_trial']
    ast.parse(Path(metadata['launch_path']).read_text(encoding='utf-8'))
    return dict(passed=True, files=len(hashes), actual_laps=1, actual_duration_s=155,
                scope='prepared source/configuration only; not runtime or driving verification')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('name')
    args = parser.parse_args()
    print(json.dumps(verify(Path(__file__).resolve().parent, args.name)))
