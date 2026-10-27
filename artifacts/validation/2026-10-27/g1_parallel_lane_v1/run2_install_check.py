"""run2 전 symlink 설치본이 현재 소스를 가리키는지 확인한다(읽기 전용).

소스가 run1 빌드 때와 같으므로 재빌드 대신 이 검사로 실행 경로의 일치를 확인한다. ROS 환경을 읽은 셸에서 실행한다.
"""
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
SRC = (ROOT/'src').resolve()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


checks, details = {}, {}
package_link = ROOT/'build/arena_autonomy/arena_autonomy'
checks['autonomy_build_link_to_src'] = Path(os.path.realpath(package_link)) == SRC/'arena_autonomy/arena_autonomy'
share = ROOT/'install/arena_bringup/share/arena_bringup'
links = [p for p in share.rglob('*') if p.is_symlink()]
# CMake 생성 파일은 build/를 가리키는 것이 정상이다. 모든 링크는 실제 파일이어야 하고, launch/config는 src를 가리켜야 한다.
dangling = [p.relative_to(ROOT).as_posix() for p in links if not Path(os.path.realpath(p)).is_file()]
runtime = [p for p in links if p.relative_to(share).parts[0] in ('launch', 'config')]
outside = [p.relative_to(ROOT).as_posix() for p in runtime if not Path(os.path.realpath(p)).is_relative_to(SRC)]
checks['bringup_links_resolve'] = bool(links) and not dangling
checks['bringup_launch_config_links_to_src'] = bool(runtime) and not outside
details['bringup_links'] = len(links)
details['bringup_launch_config_links'] = len(runtime)
details['dangling_links'] = dangling
details['launch_config_outside_src'] = outside
for name in ('arena_autonomy.parallel_lane_trial', 'arena_autonomy.local_follow', 'arena_autonomy.local_pursuit'):
    module = importlib.import_module(name)
    real = Path(os.path.realpath(module.__file__))
    expected = SRC/'arena_autonomy'/Path(*name.split('.'))
    expected = expected.with_suffix('.py')
    details[name] = {'imported': real.as_posix(), 'sha256': digest(real)}
    checks[f'{name}_from_src'] = real == expected
launch = share/'launch/g1_follow.launch.py'
checks['g1_follow_launch_matches_src'] = digest(launch) == digest(SRC/'arena_bringup/launch/g1_follow.launch.py')
report = {'scope': 'symlink install consistency before run2; not a build or driving result',
          'checks': checks, 'details': details, 'passed': all(checks.values())}
(HERE/'run2_install_check.json').write_text(json.dumps(report, ensure_ascii=False, indent=1)+'\n',
                                            encoding='utf-8', newline='\n')
print(json.dumps({'passed': report['passed'], 'checks': checks}))
sys.exit(0 if report['passed'] else 1)
