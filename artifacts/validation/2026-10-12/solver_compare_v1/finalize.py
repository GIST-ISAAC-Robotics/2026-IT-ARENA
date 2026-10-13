"""닫힌 실행/분석 산출물과 소스 증거를 봉인한다. stdout 로그 자체는 포함하지 않는다."""
import json
from pathlib import Path
import subprocess
import zipfile

from analyze_evaluator_gc import sha, source_snapshot_valid

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
analysis = json.loads((BASE/'analysis_v3.json').read_text())
diagnosis = json.loads((BASE/'diagnosis.json').read_text())
registration = json.loads((BASE/'registered_manifest.json').read_text())
sources = {**registration['sources'], **analysis['analysis_source_sha256']}
for p in (BASE/'finalize.py', BASE/'analysis.sh', BASE/'analysis_v2.sh', BASE/'analysis_v3.sh',
          BASE/'repeat_dantzig.py', BASE/'repeat_dantzig.sh', BASE/'diagnose.py',
          ROOT/'tests/test_solver_compare_analysis.py'):
    sources[p.relative_to(ROOT).as_posix()] = sha(p)
checks = {'comparison_rejection_preserved': analysis['comparison_valid'] is False and analysis['input_comparison_valid'] is False,
          'cancellation_diagnosis': all(diagnosis['checks'].values()),
          'sources_unchanged': all(sha(ROOT/p) == v for p, v in sources.items()),
          'registration_zip': source_snapshot_valid(BASE/'registered_source.zip', registration['sources']),
          'drivetrain_binary_unchanged': sha(ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so') == registration['plugin_sha256']}
raw = {}
for name in ('d1_dantzig', 'd2_dantzig', 'p1_pgs'):
    report = json.loads((BASE/name/'report.json').read_text())
    checks[name+'_input_and_cleanup'] = all(report.get(k) is True for k in
        ('input_valid', 'shutdown_clean', 'stop_acknowledged', 'source_unchanged'))
    checks[name+'_physical_outcome_preserved'] = report.get('stopped') is (name != 'p1_pgs') and report.get('passed') is (name != 'p1_pgs')
    status = json.loads((BASE/f'status_{name}.json').read_text())
    checks[name+'_residual_empty'] = status['cases'][0]['residual_after'] == []
    for p in (BASE/name).iterdir():
        if p.suffix == '.jsonl' and p.stem != 'plugin_command_events' or p.suffix == '.i64':
            rel = p.relative_to(ROOT).as_posix()
            raw[rel] = sha(p)
            ignored = subprocess.run(['git', 'check-ignore', '-q', '--', rel], cwd=ROOT).returncode == 0
            tracked = bool(subprocess.check_output(['git', 'ls-files', '--', rel], cwd=ROOT).strip())
            checks['raw_ignore_'+name+'_'+p.name] = ignored and not tracked
libraries = [Path('/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim8-wheel-slip-system.so.8.11.0'),
    Path('/opt/ros/jazzy/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so.7.6.0'),
    Path('/opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart.so.6.13.2')]
checks['engine_binaries_unchanged'] = [sha(p) for p in libraries] == [
    'fea86fe9d96da6a216ddb37acd6b5061cb2a2bcf3fd68591f1d5d013405f9044',
    '9f4ae6736fc6d4751b937ca7c14b056baf423553e20530f6d10d3ec8fd28bdd9',
    'cb4f008188d1ea13cba9b43470c9d46cf819e8df0e4fdc818fa1e4f81b396426']
artifacts = {p.relative_to(BASE).as_posix(): sha(p) for p in BASE.rglob('*') if p.is_file()
             and p.suffix not in ('.jsonl', '.i64') and p.name not in ('final_manifest.json', 'final_source.zip')}
with zipfile.ZipFile(BASE/'final_source.zip', 'x', zipfile.ZIP_DEFLATED) as z:
    for p in sources:
        z.write(ROOT/p, p)
with (BASE/'final_manifest.json').open('x') as f:
    json.dump({'scope': 'evidence closure only, not a physics or stopped acceptance',
               'checks': checks, 'evidence_finalized': all(checks.values()), 'physical_run_outcomes':
               {'d1_dantzig': True, 'd2_dantzig': True, 'p1_pgs': False},
               'input_comparison_valid': analysis['input_comparison_valid'], 'sources': sources, 'raw_sha256': raw,
               'artifact_sha256': artifacts, 'libraries_sha256': {str(p): sha(p) for p in libraries},
               'final_source_zip_sha256': sha(BASE/'final_source.zip'),
               'upstream_sources': {name: {'url': url, 'sha256': sha(BASE/name)} for name, url in (
                    ('WorldFeatures.cc', 'https://raw.githubusercontent.com/gazebosim/gz-physics/gz-physics7_7.6.0/dartsim/src/WorldFeatures.cc'),
                    ('PgsBoxedLcpSolver.cpp', 'https://raw.githubusercontent.com/dartsim/dart/v6.13.2/dart/constraint/PgsBoxedLcpSolver.cpp'),
                    ('PgsBoxedLcpSolver.hpp', 'https://raw.githubusercontent.com/dartsim/dart/v6.13.2/dart/constraint/PgsBoxedLcpSolver.hpp'))}}, f, indent=2)
print(json.dumps({'evidence_finalized': all(checks.values()), 'failed_checks': [k for k, v in checks.items() if not v],
                  'source_files': len(sources), 'local_raw_files': len(raw)}, indent=2))
raise SystemExit(0 if all(checks.values()) else 1)
