"""원 r407 실패를 읽기만 하고 revision_v2에 좁은 근거/순수 회귀 fixture를 만든다."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ORIGINAL = HERE.parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def write_new(path, value):
    with path.open('x', encoding='utf8') as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def main():
    original_manifest = read(ORIGINAL / 'manifest.json')
    assert digest(ORIGINAL / 'manifest.json') == '5d0718dc4246939383399211f29351a37ab2f572e531cbb22659b3eab59c8d2d'
    changes = [n for n, h in original_manifest['files'].items() if digest(ORIGINAL / n) != h]
    assert not changes, changes
    result_root = ORIGINAL / 'verify_v1/extracted/results'
    names = ['report.json', 'states.jsonl', 'http.jsonl', 'results.jsonl', 'boundaries.jsonl',
             'race_commands.jsonl', 'ops/ops.jsonl', 'host/summary.json', 'traceback.txt', 'host.log', 'ops.log']
    evidence = {str((result_root / n).relative_to(ORIGINAL)): digest(result_root / n) for n in names}
    execution_path = ORIGINAL / 'verify_v1/extracted/execution.json'
    evidence[str(execution_path.relative_to(ORIGINAL))] = digest(execution_path)
    rows = {n: [json.loads(line) for line in (result_root / (n + '.jsonl')).read_text(encoding='utf8').splitlines()]
            for n in ('states', 'http', 'results', 'boundaries', 'race_commands', 'ops/ops')}
    report = read(result_root / 'report.json')
    execution = read(execution_path)
    stop = next(r for r in rows['results'] if r['request'] == 'r407-stop')
    resume = next(r for r in rows['results'] if r['request'] == 'r407-resume')
    resume_http = next(r for r in rows['http'] if r['request']['request'] == 'r407-resume')
    old = rows['states'][16]; new = rows['states'][17]
    stop_entry = report['checks']['button_stop']['operator_entry']
    later_sample = next(r for r in rows['ops/ops'] if r['row'] == 'sample' and r.get('intervention') == stop['intervention'])
    rejection_event = next(r for r in rows['ops/ops'] if r.get('request') == 'r407-resume'
                           and 'intervention_mismatch' in r.get('text', ''))
    assert report['passed'] is False and report['error'] == 'TimeoutError: resume input boundary'
    assert stop['accepted'] is True and stop['intervention'] == 2
    assert resume['accepted'] is False and resume['reason'] == 'intervention_mismatch'
    assert resume_http['request']['view']['intervention'] == old['race']['intervention'] == 1
    assert old['race']['resume_start_ready'] is True and old['race']['latch'] == 'process_start'
    assert new['race']['intervention'] == 2 and new['race']['latch'] == 'explicit_stop'
    assert len(rows['boundaries']) == 1 and rows['boundaries'][0]['reason'] == 'restore_race'
    assert all(r['race']['resume'] is None for r in rows['states'])
    accepted_at = stop_entry['policy']['wall_s']
    resume_at = resume_http['response']['received_wall']
    assert accepted_at < resume_at < rejection_event['wall_s'] < later_sample['wall_s']
    assert execution['source_files'] == 208 and not execution['source_changes'] and not execution['preparation_changes']
    assert [s['returncode'] for s in execution['steps']] == [0, 0, 1]
    assert not report['external_project_imports'] and not report['cleanup_errors'] and not report['forced']
    assert report['host_shutdown']['matching_stop_tx_receipt'] and report['host_shutdown']['stop_ack_confirmed']
    fixture = dict(schema='r407_original_failure_projection_v1',
        note='Original failed run rows; later state is not a successful resume. HTTP display projection in tests only.',
        sources=evidence, state_row_indices_zero_based=[16, 17],
        state_before_stop=old, state_after_stop=new, stop_reply=stop, resume_rejection=resume,
        resume_http=resume_http, stop_entry=stop_entry)
    write_new(HERE / 'tests/failure_fixture.json', fixture)
    audit = dict(schema='r407_original_failure_audit_v2', original_pipeline_passed=False,
        original_execution_completed=execution['execution_completed'], original_error=report['error'],
        original_manifest_sha256=digest(ORIGINAL / 'manifest.json'), original_preparation_changes=changes,
        evidence_sha256=evidence, classification='harness stale ready state; correct intervention_mismatch rejection',
        stop_reply=stop, resume_request=resume_http['request'], resume_rejection=resume,
        state_rows=dict(before=16, after=17, index_basis=0, old_ready=True, old_intervention=1, new_intervention=2),
        timeline_monotonic_s=dict(stop_accepted=accepted_at, stop_host_ack_recorded=stop_entry['mcu_steps'][0]['wall_s'],
            resume_requested=resume_at, resume_rejected=rejection_event['wall_s'], new_status_received=later_sample['wall_s']),
        intervals_ms=dict(stop_accept_to_status=(later_sample['wall_s']-accepted_at)*1000,
            stop_accept_to_resume=(resume_at-accepted_at)*1000, resume_before_new_status=(later_sample['wall_s']-resume_at)*1000),
        input_boundaries=rows['boundaries'], no_resume_phase_observed=True,
        native_and_preflight_returncodes=[s['returncode'] for s in execution['steps'][:2]],
        frozen_source_files=208, source_changes=execution['source_changes'], preparation_changes=execution['preparation_changes'],
        external_project_imports=report['external_project_imports'], host_shutdown=report['host_shutdown'],
        children=report['children'], worker_cleanup=report['worker_cleanup'], vision_exitcode=report['vision_exitcode'],
        cleanup_errors=report['cleanup_errors'], physical_actuation_verified=False, physical_stop_verified=False,
        new_ros_run_performed=False, common_code_changes=0)
    write_new(HERE / 'original_audit.json', audit)
    print(json.dumps(dict(original_passed=False, cause=audit['classification'], evidence_files=len(evidence))))


if __name__ == '__main__':
    main()
