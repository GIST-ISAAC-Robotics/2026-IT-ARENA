"""동결 전 r499 대비 차이와 현재 공통 SHA 대조. 실행 등록/동결·장치 접근 없음."""
import ast
import hashlib
import json
from pathlib import Path
import re

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = A.parent / 'rear_overtake_current_geometry_r499_v1'
read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    cfg, before = read(A / 'config.json'), read(OLD / 'config.json')
    assert cfg == read(A / 'expected_config.json')
    diff = sorted(k for k in set(cfg) | set(before) if cfg.get(k) != before.get(k))
    assert diff == ['launch', 'registered_at', 'scope'], diff
    launch = dict(cfg['launch']); launch['entry_g1'] = before['launch']['entry_g1']
    assert launch == before['launch']
    old = read(OLD / 'prepared.json')
    expected = [x.replace(OLD.name, A.name) for x in old['args']]
    assert read(A / 'arguments.json') == expected
    provenance = read(A / 'recipe_provenance.json')
    for item in provenance['files']:
        if item['byte_exact']:
            assert sha(A / item['path']) == sha(OLD / item['path']) == item['before_sha256'], item['path']
        else:
            assert sha(A / item['path']) == item['after_sha256'], item['path']
    for name, digest in provenance['original_evidence'].items():
        assert sha(ROOT / name) == digest, name
    selection = read(A / 'source_selection.json')
    allowed = {x['path']: x for x in selection['control_changes']}
    common = {p: h for p, h in old['frozen_inputs'].items()
              if p.startswith(('src/', 'scripts/', 'tests/', 'config/')) or 'old_signal_610_r326_v1/' in p}
    delta = {}
    for p, h in common.items():
        now = sha(ROOT / p)
        if now != h:
            delta[p] = now
    unexpected = sorted(set(delta) - set(allowed) - set(selection['noncontrol_changes']))
    assert not unexpected, unexpected
    for p, item in allowed.items():
        assert item['before_sha256'] == common[p] and delta.get(p) == item['after_sha256'] == sha(ROOT / p), p
    for p in selection['additional_tests']:
        assert p not in common and (ROOT / p).is_file(), p
    native = read(A.parent / 'traffic_current_geometry_r496_v1/native_v1/receipt.json')
    native_changed = [p for p, h in native['production_source_hashes'].items() if sha(ROOT / p) != h]
    # control 후보가 native166 production 목록에 포함될 수 있다. .so는 그대로이고 재빌드하지 않는다.
    assert set(native_changed) <= set(allowed), native_changed
    library = 'src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so'
    assert sha(ROOT / library) == native['native_sha256'] == common[library]
    assert read(A / 'geometry_right_side.json')['passed']
    for p in A.rglob('*.py'):
        ast.parse(p.read_bytes())
    for p in [*A.rglob('*.sh'), *A.rglob('*.bash')]:
        assert b'\r' not in p.read_bytes(), p
    if (A / 'PLAN.md').exists():
        missing = [t for t in re.findall(r'\]\(([^)#]+)', (A / 'PLAN.md').read_text(encoding='utf-8'))
                   if not (A / t).exists()]
        assert not missing, missing
    assert not any((A / n).exists() for n in ('prepared.json', 'sources.zip', 'registered.json', 'run1'))
    result = dict(passed=True, common_files_compared=len(common),
        control_delta={p: [allowed[p]['before_sha256'], allowed[p]['after_sha256']] for p in allowed},
        noncontrol_delta={p: [common[p], delta[p]] for p in selection['noncontrol_changes'] if p in delta},
        additional_tests={p: sha(ROOT / p) for p in selection['additional_tests']},
        config_changed_keys_vs_r499=diff, case_actor_acceptance_unchanged=True, arguments_only_path_renamed=True,
        native_production_changed_only_control=sorted(native_changed), native166_not_rerun=True, native_library_sha256=sha(ROOT / library), new_build=False,
        geometry_reused_r499=True, original_evidence_checked=len(provenance['original_evidence']),
        common_frozen=False, actual_simulation_started=False, wsl_used_for_this_check=False)
    out = A / 'preparation_receipt.json'
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: result[k] for k in ('passed', 'common_files_compared', 'noncontrol_delta', 'config_changed_keys_vs_r499')}))


if __name__ == '__main__':
    main()
