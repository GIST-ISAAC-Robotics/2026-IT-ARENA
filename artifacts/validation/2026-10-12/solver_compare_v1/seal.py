"""첫 마감에서 발견한 GC/느린 콜백 원시 로그 제외 누락만 보완 확인. 실패 manifest는 보존."""
import json
from pathlib import Path
import subprocess
from analyze_evaluator_gc import sha, source_snapshot_valid

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
previous = json.loads((BASE/'final_manifest.json').read_text())
expected = {f'raw_ignore_{n}_{f}.jsonl' for n in ('d1_dantzig', 'd2_dantzig', 'p1_pgs')
            for f in ('gc_events', 'slow_callbacks')}
checks = {'only_expected_ignore_checks_failed': {k for k, v in previous['checks'].items() if not v} == expected,
          'sources_unchanged': all(sha(ROOT/p) == h for p, h in previous['sources'].items()),
          'closed_artifacts_unchanged': all(sha(BASE/p) == h for p, h in previous['artifact_sha256'].items()),
          'raw_unchanged': all(sha(ROOT/p) == h for p, h in previous['raw_sha256'].items()),
          'final_source_zip': source_snapshot_valid(BASE/'final_source.zip', previous['sources']),
          'raw_all_ignored_untracked': all(subprocess.run(['git', 'check-ignore', '-q', '--', p], cwd=ROOT).returncode == 0
              and not subprocess.check_output(['git', 'ls-files', '--', p], cwd=ROOT).strip() for p in previous['raw_sha256']),
          'failed_physics_and_comparison_preserved': previous['physical_run_outcomes']['p1_pgs'] is False
              and previous['input_comparison_valid'] is False}
result = {'evidence_finalized': all(checks.values()), 'checks': checks,
    'scope': 'evidence closure only; PGS stopped=false and unmatched applied-input comparison remain failures',
    'previous_manifest_sha256': sha(BASE/'final_manifest.json'), 'gitignore_sha256': sha(ROOT/'.gitignore'),
    'seal_source_sha256': sha(Path(__file__))}
with (BASE/'seal.json').open('x') as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['evidence_finalized'] else 1)
