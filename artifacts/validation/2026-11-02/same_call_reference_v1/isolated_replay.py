"""명시한 이관 묶음만 새 로컬 폴더에 풀어 import·판단 재생을 확인한다."""
from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
TARGET = ROOT/'build/jetson_follow_20261102_v2_extract'
ARCHIVE = ROOT/'build/jetson_follow_20261102_v2.tar.gz'
MANIFEST = HERE/'bundle_manifest.json'


def child():
    # -I로 원래 PYTHONPATH·현재 작업 폴더·사용자 site-packages를 배제한다.
    root = TARGET.resolve()
    sys.path[:0] = [str(root/'scripts'), str(root/'src/arena_autonomy'),
                   str(root/'src/arena_vehicle_interface')]
    import profile_follow_trace
    from arena_autonomy.follow_trace import replay_events
    manifest = json.loads(MANIFEST.read_text())
    rows = []
    for name in manifest['files']:
        if not (name.startswith('artifacts/') and name.endswith('.jsonl')):
            continue
        path = root/name
        with path.open(encoding='utf-8') as stream:
            report = replay_events(profile_follow_trace.read_events(path))
            report['output_mismatches'] = report.get('mismatches', [])
        rows.append(dict(path=name, **{k:v for k,v in report.items() if k not in ('rows','output_mismatches')},
                         mismatch_count=len(report['output_mismatches'])))
    modules = {name:module.__file__ for name,module in sys.modules.items()
               if (name.startswith(('arena_autonomy.','arena_vehicle_interface.'))
                   or name == 'profile_follow_trace') and getattr(module,'__file__',None)}
    isolated = bool(modules) and all(Path(path).resolve().is_relative_to(root) for path in modules.values())
    hashes = all(sha256((root/name).read_bytes()).hexdigest()==item['sha256']
                 for name,item in manifest['files'].items())
    result = dict(passed=isolated and hashes and all(row['passed'] for row in rows),
                  modules_from_extract_only=isolated, extracted_files_unchanged=hashes,
                  modules=modules, replays=rows, remote_execution=False,
                  scope='로컬 v2 격리 묶음 import와 저장 판단 일치만 확인. 추가 성능계측/Jetson/ROS 시험 아님.')
    with (HERE/'isolated_replay.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k!='modules'},ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    if '--child' in sys.argv:
        raise SystemExit(child())
    sys.path.insert(0,str(ROOT/'scripts'))
    from verify_follow_profile_bundle import verify
    manifest = json.loads(MANIFEST.read_text())
    assert verify(ARCHIVE,manifest)['passed']
    assert TARGET.resolve().is_relative_to((ROOT/'build').resolve())
    TARGET.mkdir(exist_ok=False)
    with tarfile.open(ARCHIVE,'r:gz') as archive:
        archive.extractall(TARGET,filter='data')
    env = {k:v for k,v in os.environ.items()
           if not k.startswith(('PYTHON','ROS','AMENT','COLCON','RMW','CYCLONEDDS','FASTRTPS'))}
    raise SystemExit(subprocess.run([sys.executable,'-I','-B',str(Path(__file__).resolve()),'--child'],
                                   cwd=TARGET,env=env).returncode)
