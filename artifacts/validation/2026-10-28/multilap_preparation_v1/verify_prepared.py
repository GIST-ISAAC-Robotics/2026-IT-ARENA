"""5랩 요청과 보존 소스를 실행 전에 대조한다. 주행 성공 검사가 아니다."""
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
        assert len(members) == len(set(members)), 'duplicate source snapshot member'
        assert all(sha(archive.read(path)) == value == sha((root/path).read_bytes())
                   for path, value in hashes.items()), 'source mismatch'
    config = json.loads((base/'traffic_config.json').read_text(encoding='utf-8'))
    case = next(c for c in config['cases'] if c['name'] == metadata['route']['case'])
    assert case['duration_s'] == metadata['route']['duration_s'] == 685.
    assert config['acceptance'][case['name']]['minimum_laps'] == metadata['route']['minimum_laps'] == 5
    assert metadata['route']['config'] == str(base/'traffic_config.json')
    assert metadata['route']['actor_overlay'] == str(base/'actor_overlay.json')
    assert metadata['launch_arguments']['speed_bump'] == 'true'
    hook = metadata['route_hook']
    assert hook['evaluation_only'] and not hook['controller_parameters_changed']
    assert hook['candidate_sha256'] == sha((base/'course_candidate.py').read_bytes())
    assert hook['original_module_sha256'] == sha((root/'scripts/g1_course_eval.py').read_bytes())
    assert (base/'actor_overlay.json').relative_to(root).as_posix() in hashes
    assert (base.parent/'bump_traffic_v1/run.py').relative_to(root).as_posix() in hashes
    assert (base.parent/'bump_traffic_v1/analyze.py').relative_to(root).as_posix() in hashes
    ast.parse(Path(metadata['launch_path']).read_text(encoding='utf-8'))
    return dict(passed=True, source_files=len(hashes), minimum_laps=5, duration_s=685.,
        scope='prepared source and requested evaluation scope; not runtime or driving proof')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('name')
    args = parser.parse_args()
    print(json.dumps(verify(Path(__file__).resolve().parent, args.name)))
