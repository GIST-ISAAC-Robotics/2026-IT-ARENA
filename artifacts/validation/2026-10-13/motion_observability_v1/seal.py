"""읽은 원시 입력/운영 소스 보존과 문서 링크를 마감 확인한다. 주행 합격 판정이 아니다."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote
import zipfile

from analyze_motor_observability import ROOT, RUNS, sha

BASE = Path(__file__).resolve().parent
analysis = json.loads((BASE/'analysis.json').read_text(encoding='utf-8'))
checks = {}
checks['analysis_code_unchanged'] = sha(ROOT/'scripts/analyze_motor_observability.py') == analysis['analysis_sha256']
checks['sources_unchanged_since_analysis'] = all(sha(ROOT/path) == value
    for path, value in analysis['current_source_sha256'].items())
checks['inputs_unchanged'] = all(sha(ROOT/RUNS[name]/path) == value
    for name, result in analysis['runs'].items() for path, value in result['input_sha256'].items())
core = [p for p in analysis['current_source_sha256'] if p.startswith('src/') and
        p.rsplit('/', 1)[-1] in ('actuation_contract.py', 'lidar_motion.py', 'lidar_motion_ros.py',
                                 'local_follow.py', 'local_pursuit.py', 'scene_motion.py')]
baseline = {}
for name, path in RUNS.items():
    with zipfile.ZipFile(ROOT/path/'source_snapshot.zip') as archive:
        baseline[name] = {p: hashlib.sha256(archive.read(p)).hexdigest() == sha(ROOT/p) for p in core}
checks['six_core_sources_match_all_original_runs'] = all(all(r.values()) for r in baseline.values())
checks['regression_summary'] = '275 passed' in (BASE/'tests.log').read_text(encoding='utf-8')
changed_runtime = subprocess.run(['git', 'diff', '--name-only', 'HEAD', '--', 'src', 'config'],
    cwd=ROOT, text=True, capture_output=True, check=True).stdout.splitlines()
checks['tracked_runtime_unchanged_from_head'] = not changed_runtime
docs = [
    ROOT/'docs/PROJECT_CONTEXT.md', ROOT/'docs/README.md',
    ROOT/'docs/autonomy/MOTOR_MOTION_OBSERVABILITY_2026_10_13.md',
    ROOT/'docs/autonomy/MOTOR_ENCODER_VALIDATION_2026_10_02.md',
    ROOT/'docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md',
    ROOT/'docs/hardware/ACTUATION_CONTRACT_2026_09_22.md',
    ROOT/'docs/hardware/ACTUATION_ROS_2026_09_22.md',
    ROOT/'docs/sensors/LIDAR_MOTION_IMPLEMENTATION.md',
    ROOT/'docs/reports/DEVELOPMENT_PLAN_2026_10_04.md',
    ROOT/'docs/activity/2026-10-13.md', BASE/'README.md',
]
missing_links = []
for path in docs:
    for target in re.findall(r'\]\(([^)]+)\)', path.read_text(encoding='utf-8')):
        target = target.split('#', 1)[0]
        if not target or '://' in target:
            continue
        if not (path.parent/unquote(target)).exists():
            missing_links.append({'document': str(path.relative_to(ROOT)), 'target': target})
checks['local_file_links_exist'] = not missing_links
checks['diff_check'] = subprocess.run(['git', 'diff', '--check'], cwd=ROOT, capture_output=True).returncode == 0
members = [ROOT/p for p in analysis['current_source_sha256']] + [ROOT/'scripts/analyze_motor_observability.py',
           BASE/'run.sh', BASE/'seal.py']
with zipfile.ZipFile(BASE/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
    for path in members:
        archive.write(path, path.relative_to(ROOT))
result = {'scope': '증거/문서 보존 검사. 기존 G1 실패·PGS 미정지·관측 한계를 통과로 바꾸지 않는다.',
          'checks': checks, 'evidence_checks_passed': all(checks.values()),
          'core_source_comparison': baseline, 'missing_file_links': missing_links,
          'changed_runtime': changed_runtime,
          'artifact_sha256': {p.name: sha(p) for p in BASE.iterdir() if p.is_file() and p.name != 'seal.json'},
          'document_sha256': {str(p.relative_to(ROOT)): sha(p) for p in docs}}
with (BASE/'seal.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
print(json.dumps({'checks': checks, 'missing_links': missing_links}, ensure_ascii=False, indent=2))
raise SystemExit(0 if all(checks.values()) else 1)
