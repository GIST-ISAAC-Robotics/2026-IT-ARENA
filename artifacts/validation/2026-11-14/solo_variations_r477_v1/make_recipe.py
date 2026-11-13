"""r460의 두 변형만 r475 명령/소스 기준으로 준비한다. Windows 파일 작업만 수행."""
import ast
import difflib
import hashlib
import json
from pathlib import Path
import shutil

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = ROOT / 'artifacts/validation/2026-11-13/solo_variations_r460_v1'
BASE = A.parent / 'solo_full_lap_r475_v1'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not (A/'recipe_copy.json').exists()
    oldrel, newrel = OLD.relative_to(ROOT).as_posix(), A.relative_to(ROOT).as_posix()
    exact = ['solo_runner.py', 'solo_evaluation.py', 'solo_finalize.py', 'facility_geometry.py',
        'facilities_only.sdf', 'expected_header.json', 'audit_runtime.py', 'header_comparison.py',
        'course.launch.py', 'launch/demo.launch.py', 'launch/local_pursuit.launch.py']
    paths = [*exact, 'prepare.py', 'prepare_execution.py', 'variation_launch.py', 'launch_adapter.py',
        'runtime_preflight.py', 'static_check.py', 'run.py', 'stop_owned.py', 'launch/simulation.launch.py',
        'execution/execute_once.sh', 'execution/stop_once.sh', 'execution/observe.py',
        'execution/check_owned_processes.py']
    copied, baseline, patch = [], {}, []
    for name in paths:
        source = (BASE if name in exact else OLD)/name
        target = A/name
        target.parent.mkdir(parents=True, exist_ok=True)
        if name in exact:
            shutil.copyfile(source, target)
            baseline[name] = dict(baseline_sha256=sha(source), copy_sha256=sha(target))
            mode = 'r475_byte_exact'
        else:
            text = source.read_text(encoding='utf-8').replace(oldrel, newrel)
            text = text.replace('r460', 'r477').replace('R460', 'R477').replace('r457', 'r475')
            text = text.replace("    'start_right20_yaw_minus3': {'lateral_m': -.02, 'yaw_delta_deg': -3., 'wall_extra_width_m': 0.},\n", '')
            text = text.replace('start_left20_yaw_plus3|start_right20_yaw_minus3|wall_surface_inward10',
                                'start_left20_yaw_plus3|wall_surface_inward10')
            text = text.replace("fixture = A.parent/'old_signal_610_r326_v1'",
                                "fixture = ROOT/'artifacts/validation/2026-11-13/old_signal_610_r326_v1'")
            text = text.replace('runtime_three_variants_exact', 'runtime_two_variants_exact')
            if name == 'prepare_execution.py':
                start = text.index("    selected = {p:h for p,h in baseline['frozen_inputs'].items()")
                end = text.index('    for path in (WORLD, SCENE,', start)
                text = text[:start] + "    selected = dict(baseline['frozen_inputs'])  # r475의 1365개 동결 입력 전체\n" + text[end:]
                text = text.replace("        original_runtime_probe_failure_sha256=sha(A/'prepared/runtime_probe_initial_failure.json'),\n", '')
                text = text.replace("A/'static_v1/result.json', A/'static_check_v1.log', A/'static_check_v1.exit',",
                                    "A/'static_v1/result.json', A/'static_check_v1.log', A/'static_check_v1.exit', A/'recipe_copy.json', A/'recipe_review.patch',")
                # Gate remains original success/STOP/integrity/PID contract, linked to latest nominal.
                text = text.replace("    report = read(BASE/'run1/report.json')", "    assert all((BASE/p).is_file() for p in ('run1/report.json', 'integrity.json', 'execution_v1/owned_processes_after.json')), 'r475 final result/cleanup pending; variants remain held'\n    report = read(BASE/'run1/report.json')")
            if name == 'static_check.py':
                text = text.replace('initial_static_and_failed_probe_preserved=True', 'initial_static_manifest_preserved=True')
            target.write_text(text, encoding='utf-8', newline='\n')
            mode = 'r460_minimal_path_baseline_two_cases'
        copied.append(dict(path=name, source=source.relative_to(ROOT).as_posix(),
                           source_sha256=sha(source), prepared_sha256=sha(target), mode=mode))
        if source.read_bytes() != target.read_bytes():
            patch.extend(difflib.unified_diff(source.read_text(encoding='utf-8').splitlines(True),
                target.read_text(encoding='utf-8').splitlines(True), fromfile=source.relative_to(ROOT).as_posix(),
                tofile=target.relative_to(ROOT).as_posix()))
    config = json.loads((BASE/'config.json').read_bytes())
    config['scope'] = 'r477 two evaluation-only variations after successful r475 nominal; same control/source/geometry/sensors/speed/STOP except registered spawn or one wall surface'
    config['launch']['entry_g1'] = (A/'course.launch.py').relative_to(ROOT).as_posix()
    (A/'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    copied.append(dict(path='config.json', source=(BASE/'config.json').relative_to(ROOT).as_posix(),
        source_sha256=sha(BASE/'config.json'), prepared_sha256=sha(A/'config.json'), mode='scope_and_entry_path_only'))
    for p in A.rglob('*.py'): ast.parse(p.read_bytes())
    (A/'baseline_copy.json').write_text(json.dumps(baseline, indent=2)+'\n', encoding='utf-8')
    (A/'recipe_copy.json').write_text(json.dumps(copied, indent=2)+'\n', encoding='utf-8')
    (A/'recipe_review.patch').write_text(''.join(patch), encoding='utf-8')
    print(json.dumps(dict(prepared_files=len(copied), r475_exact_files=len(exact), cases=2,
        common_changed=False, simulation_started=False)))


if __name__ == '__main__': main()
