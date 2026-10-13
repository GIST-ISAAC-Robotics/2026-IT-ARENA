"""후처리 재계산·기존 실행/원시 보존·문서 링크·현재 라이브러리와 소스 사본 확인."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
PARENT = BASE.parent/'scheduled_surface_v1'
sys.path.insert(0, str(ROOT/'scripts'))
from analyze_contact_model import analyze

sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
if (BASE/'manifest.json').exists() or (BASE/'final_source.zip').exists():
    raise SystemExit('refusing to overwrite final audit')
prior = json.loads((PARENT/'final_manifest.json').read_text())
saved = json.loads((BASE/'analysis_final.json').read_text())
recomputed = analyze(BASE)
historical = {n: sha(PARENT/n) == digest for n, digest in prior['artifact_sha256'].items()}
runtime = {}
for name in ('s1_official', 'f1_flat'):
    report = json.loads((PARENT/name/'report.json').read_text())
    runtime[name] = {n: sha(ROOT/n) == digest for n, digest in report['source_sha256'].items()}
raw = []
for r in prior['raw']:
    path = ROOT/r['path']
    raw.append({'path': r['path'], 'unchanged': sha(path) == r['sha256'],
                'ignored': subprocess.run(['git', 'check-ignore', '--quiet', r['path']], cwd=ROOT).returncode == 0,
                'tracked': bool(subprocess.check_output(['git', 'ls-files', '--', r['path']], cwd=ROOT).strip())})
environment = json.loads((BASE/'installed_environment.json').read_text())
binary_checks = {p: sha(Path(p)) == digest for p, digest in environment['binaries'].items()}
probe = json.loads((BASE/'solver_probe.json').read_text())
probe_build = Path((BASE/'probe_build_path_v2.txt').read_text().strip())
probe_hashes = {str(probe_build/'solver_probe'): sha(probe_build/'solver_probe') if (probe_build/'solver_probe').exists() else None,
                '/opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart-external-odelcpsolver.so.6.13':
                sha(Path('/opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart-external-odelcpsolver.so.6.13'))}
docs = ['docs/PROJECT_CONTEXT.md', 'docs/activity/2026-10-12.md',
        'docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md', 'docs/simulation/VEHICLE_DYNAMICS.md']
broken = []
for name in docs:
    doc = ROOT/name
    for target in re.findall(r'\]\(([^)]+)\)', doc.read_text(encoding='utf-8')):
        if '://' in target or target.startswith('#'):
            continue
        dest = (doc.parent/unquote(target.split('#', 1)[0])).resolve()
        if dest != BASE/'manifest.json' and not dest.exists():
            broken.append([name, target])
tests = (BASE/'tests_final.log').read_text()
checks = {'recomputed_analysis_matches_saved': recomputed == saved,
          'analysis_integrity': saved['analysis_integrity_passed'],
          'historical_artifacts_unchanged': all(historical.values()),
          'runtime_sources_unchanged': all(all(v.values()) for v in runtime.values()),
          'raw_unchanged_ignored_untracked': len(raw) == 20 and all(r['unchanged'] and r['ignored'] and not r['tracked'] for r in raw),
          'current_engine_binaries_match': all(binary_checks.values()),
          'solver_probe_reproduced': probe['mechanism_reproduced'] and probe['default_primary'] == 'DantzigBoxedLcpSolver',
          'source_v1_preserved': sha(BASE/'analysis_source_v1.py') == json.loads((BASE/'analysis.json').read_text())['source_sha256']['scripts/analyze_contact_model.py'],
          'relevant_tests_passed': bool(re.search(r'\b16 passed\b', tests)),
          'document_file_links': not broken}
files = [ROOT/n for n in docs+['scripts/analyze_contact_model.py', 'tests/test_contact_model_analysis.py']]
files += [p for p in BASE.rglob('*') if p.is_file() and '__pycache__' not in p.parts
          and p.name not in ('manifest.json', 'final_source.zip', 'finalize.log')]
with zipfile.ZipFile(BASE/'final_source.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
    for p in sorted(files):
        bundle.write(p, p.relative_to(ROOT).as_posix())
result = {'passed': all(checks.values()), 'checks': checks, 'historical_artifact_checks': historical,
          'runtime_source_checks': runtime, 'raw_policy': raw, 'current_engine_binary_checks': binary_checks,
          'probe_binary_sha256': probe_hashes, 'broken_links': broken,
          'probe_binary_retained': (probe_build/'solver_probe').exists(),
          'probe_binary_limit': 'Temporary build was missing at finalization; original executable hash unavailable. Source, build log, ldd and result are retained; no rebuild claimed as historical proof.',
          'source_files_sha256': {p.relative_to(ROOT).as_posix(): sha(p) for p in sorted(files)},
          'final_source_zip_sha256': sha(BASE/'final_source.zip'),
          'limit': 'Current library hashes do not retroactively prove prior runtime engine hashes; physics-model validity is not the passed verdict.'}
with (BASE/'manifest.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(checks, indent=2))
raise SystemExit(0 if result['passed'] else 1)
