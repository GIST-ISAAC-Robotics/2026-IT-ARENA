"""통합 후 SHA와 기존 집중검사 결과를 기록한다. 실행시험을 추가하지 않는다."""
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

OUT = Path(__file__).resolve().parent
BUNDLE = OUT.parent
REPO = BUNDLE.parents[3]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


manifest = json.loads((OUT / 'integration.json').read_text())
checks = []
for row in manifest['files']:
    target = REPO / row['path']
    actual = sha(target)
    assert actual == row['stage_sha256'], row['path']
    if target.suffix == '.py':
        ast.parse(target.read_text(encoding='utf-8'), filename=str(target))
    backup = OUT / 'originals' / row['path']
    if row['original_sha256'] is not None:
        assert sha(backup) == row['original_sha256']
    checks.append({'path': row['path'], 'common_sha256': actual, 'stage_exact': True})
suite = ET.parse(OUT / 'windows_focus.xml').getroot().find('testsuite')
tests = {key: int(suite.attrib[key]) for key in ['tests', 'failures', 'errors', 'skipped']}
assert tests == {'tests': 94, 'failures': 0, 'errors': 0, 'skipped': 0}
assert json.loads((OUT / 'test_exit.json').read_text())['exit_code'] == 0
manifest['status'] = 'integrated_exact_Windows_focus_passed'
manifest['Windows_focus'] = tests
manifest['Windows_focus_exit'] = 0
manifest['post_test_stage_exact_count'] = len(checks)
manifest['prior_ROS_evidence'] = '../execution_v2/run1/result.json'
manifest['ROS_reexecuted'] = False
write(OUT / 'integration.json', manifest)
write(OUT / 'post_check.json', {'files': checks, 'all_stage_exact': True,
    'existing_original_backups_exact': True, 'python_AST_parse': 'passed',
    'common_file_write_scope': '22 listed files only'})
write(OUT / 'result.json', {
    'status': manifest['status'], 'integrated_files': 22, 'before_hash_conflicts': [],
    'after_stage_exact': 22, 'Windows_tests': tests, 'exit_code': 0,
    'import_paths': 'import_paths.json', 'WSL_used': False, 'Jetson_used': False,
    'ROS_reexecuted': False, 'native_modified': False,
    'PROJECT_CONTEXT_DEVELOPMENT_PLAN_modified': False,
    'prior_ROS_result': '../execution_v2/run1/result.json',
    'prior_ROS_cleanup_limit': 'normal runner/child cleanup passed; later independent WSL PID audit entry failed',
    'limits': ['default OFF until explicit profile selection',
               'approximate geometry and image-header time; actual print/calibration/exposure unverified',
               'existing candidate sweep/current/guard occupancy connected; marker-only new overtake initiation not implemented',
               'Windows integrated-source checks are not a new ROS or vehicle test'],
})

(OUT / 'README.md').write_text('''# r466 v2 공통 통합

승인된 **22파일의 before SHA가 모두 일치**하여 충돌 없이 v2 stage를 정확히 복사했다. 기존 10파일의 원본은 `originals/`에 보존했고 새 12파일도 manifest에 구분했다. 통합 후와 검사 후에도 22파일 모두 stage SHA와 동일하다.

공통 경로를 직접 import하는 Windows 집중 검사 **94 passed, exit0**이다. geometry/실제 합성 pixel 검출/중첩 quad 반례·CLI·TTL/clock/epoch·plan/current/기존 후보 sweep·독립 guard 본문 I/O double·기존 marker observer OFF 동작·VisionProcess 정상/고장 회수를 포함한다. `import_paths.json`에서 stage를 import하지 않았음을 확인한다. Python 파일의 AST 문법 검사도 통과했다. anyio 플러그인의 pytest API 폐기예정 경고1개는 실패가 아니다.

이번 통합에서는 WSL/ROS/Jetson/장치/Gazebo를 실행하지 않았다. [기존 v2 실제 ROS 대표](../execution_v2/run1/result.json)를 재실행하지 않았으며 이관 이후 새 ROS 검증으로 쓰지 않는다. 해당 실행의 정상 runner/child 종료와, 후속 별도 WSL PID 감사 진입 실패는 원 기록 그대로다.

기능은 명시 profile이 없으면 OFF다. 근사 K/FOV·CAD와 원 image stamp 계약을 유지하고 실제 인쇄물 크기/장착 교정/노출 시각의 정확도를 확인한 것으로 쓰지 않는다. 별도 점유는 wall fit/free 입력에 섞지 않는다. 현재 감속·기존 추월 후보 sweep/current/독립 guard까지 연결하며, 마커만으로 새 추월 후보를 시작하는 기능은 아직 없다.

주 담당의 `pass_release_trial.py`, `race_runtime_policy/observation_policy.py`, `current_region.py`, 새 current-route 모듈과 native에는 쓰지 않았다. `PROJECT_CONTEXT.md`, `DEVELOPMENT_PLAN.md`, 활동 문서도 수정하지 않았다. 이번 작업이 변경한 공통 파일은 [integration.json](integration.json)의22개뿐이다.

근거: [통합 영수증](integration.json), [검사 후 SHA](post_check.json), [기계 결과](result.json), [Windows 로그](windows_focus.log), [JUnit](windows_focus.xml). 원 stage/ROS source archive와 v1 실패는 보존했다. 후속 통합·주행 자원은 root 소유다.
''', encoding='utf-8')

for name in ['README.md', 'RESULT.md']:
    path = BUNDLE / name
    text = path.read_text(encoding='utf-8')
    banner = '**후속 공통 통합 완료:** [integration_v1](integration_v1/README.md)에서22파일 SHA 일치·Windows94검사 통과를 확인했다. 아래 v2 대표 당시의 공통 미통합 표기는 당시 상태이며, ROS/장치 검증을 추가한 것은 아니다.\n\n'
    head, rest = text.split('\n', 1)
    path.write_text(head + '\n\n' + banner + rest.lstrip('\n'), encoding='utf-8')
for name in ['latest_revision.json', 'result.json']:
    path = BUNDLE / name
    value = json.loads(path.read_text())
    value['status'] = manifest['status']
    value['integration'] = 'integration_v1/result.json'
    if name == 'result.json':
        value['no_common_edits'] = False
        value['common_edits_scope'] = 'integration_v1/integration.json: exact 22 approved files'
    write(path, value)
print('SEALED 22 exact, 94 passed, no WSL/Jetson/ROS rerun')
