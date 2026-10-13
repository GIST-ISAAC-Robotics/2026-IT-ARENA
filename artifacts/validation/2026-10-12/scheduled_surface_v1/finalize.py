"""노면 대조 마감: 실행판 무변경, 분석판/검증·원시 근거와 제외 정책 보존."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
registered = json.loads((BASE/'registered_manifest.json').read_text())
analysis = json.loads((BASE/'analysis_final.json').read_text())
runtime = {p: sha(ROOT/p) == value for p, value in registered['sources'].items()}
postprocessing = {p: sha(ROOT/p) == value for p, value in analysis['analysis_source_sha256'].items()}
raw_names = ('wheel_contacts.jsonl', 'braking_diagnostics.jsonl', 'requested_commands.jsonl',
             'safe_commands.jsonl', 'actuation_status.jsonl', 'output_status.jsonl',
             'scheduled_sent.jsonl', 'scheduled_events.jsonl', 'scheduled_states.jsonl', 'evaluator_timing.i64')
raw = []
full_runtime = {}
for name in ('s1_official', 'f1_flat'):
    report = json.loads((BASE/name/'report.json').read_text())
    full_runtime[name] = {p: sha(ROOT/p) == value for p, value in report['source_sha256'].items()}
    for file in raw_names:
        path = BASE/name/file
        raw.append({'path': path.relative_to(ROOT).as_posix(), 'sha256': sha(path),
            'ignored': subprocess.run(['git', 'check-ignore', '--quiet', str(path)], cwd=ROOT).returncode == 0,
            'tracked': bool(subprocess.check_output(['git', 'ls-files', '--', str(path)], cwd=ROOT).strip())})
files = set(registered['sources']) | set(postprocessing) | {
    'scripts/analyze_scheduled_surface.py', 'tests/test_scheduled_surface_analysis.py',
    'docs/PROJECT_CONTEXT.md', 'docs/activity/2026-10-12.md', 'docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md',
    '.gitignore', '.gitattributes', str(Path(__file__).relative_to(ROOT))}
docs = [ROOT/p for p in files if p.startswith('docs/')]
broken_links = []
for doc in docs:
    for target in re.findall(r'\]\(([^)]+)\)', doc.read_text(encoding='utf-8')):
        if '://' in target or target.startswith('#'):
            continue
        path = (doc.parent/unquote(target.split('#', 1)[0])).resolve()
        if path != BASE/'final_manifest.json' and not path.exists():
            broken_links.append([doc.relative_to(ROOT).as_posix(), target])
with zipfile.ZipFile(BASE/'final_source.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
    for file in sorted(files):
        bundle.write(ROOT/file, file)
checks = {'registered_sources_unchanged': all(runtime.values()), 'analysis_sources_unchanged': all(postprocessing.values()),
          'full_run_sources_still_unchanged': all(all(values.values()) for values in full_runtime.values()),
          'historical_source_reference_unchanged': sha(ROOT/analysis['source_reference']['path']) == analysis['source_reference']['sha256'],
          'document_file_links': not broken_links,
          'plugin_unchanged': sha(ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so') == registered['plugin_sha256'],
          'raw_ignored_untracked': len(raw) == 20 and all(r['ignored'] and not r['tracked'] for r in raw),
          'comparison_valid': analysis['surface_comparison_valid']}
result = {'checks': checks, 'passed': all(checks.values()), 'registered_source_checks': runtime,
          'analysis_source_checks': postprocessing, 'raw': raw, 'final_source_sha256': sha(BASE/'final_source.zip'),
          'full_run_source_checks': full_runtime, 'broken_links': broken_links,
          'final_files': {p: sha(ROOT/p) for p in sorted(files)},
          'artifact_sha256': {p.relative_to(BASE).as_posix(): sha(p) for p in BASE.rglob('*')
                            if p.is_file() and p.name not in raw_names
                            and p.name not in ('final_manifest.json', 'finalize.log')}}
with (BASE/'final_manifest.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
print(json.dumps(checks, indent=2))
raise SystemExit(0 if result['passed'] else 1)
