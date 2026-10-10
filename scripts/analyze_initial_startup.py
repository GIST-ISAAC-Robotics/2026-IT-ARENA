"""초기 삽입 대조의 근거·형상 보존과 시작 시점만 감사한다. 주행 결정성 판정은 하지 않는다."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from analyze_evaluator_gc import run_result
from analyze_apply_trace import trace_summary
from analyze_apply_trace_v2 import first_application_difference
from evaluation_startup import audit_startup, graph_ready, topic_connections, TRACE_TOPIC, EXPECTED_GZ_PUBLISHERS

ROOT = Path(__file__).resolve().parents[1]
PLUGIN_SHA = 'c48d4a99e42ba11cdf493439bf665495131aba89075c7e378a5c275597341f78'


def normalized_xml(element):
    element = copy.deepcopy(element)
    for node in element.iter():
        if node.text:
            node.text = re.sub(r'/tmp/arena_world_[^/]+/', '/tmp/RUNTIME/', node.text.strip()) or None
        if node.tail: node.tail = node.tail.strip() or None
    return ET.tostring(element)


def world_and_model(run, report):
    world = ET.parse(run/'runtime_world.sdf').getroot()
    vehicle = ET.parse(run/'runtime_vehicle.sdf').getroot().find('model')
    embedded = world.findall('world/model[@name="arena_car"]')
    mode = report['evaluation_startup']['mode']
    checks = {'model_count': len(embedded) == (1 if mode == 'embedded_paused' else 0)}
    if mode == 'embedded_paused' and len(embedded) == 1:
        model = embedded[0]
        pose = model.find('pose')
        try:
            values = [float(x) for x in pose.text.split()]
        except (AttributeError, ValueError): values = []
        spawn = report['spawn']
        checks['pose_matches'] = values == [spawn['x'], spawn['y'], .01, 0., 0., spawn['yaw_rad']]
        if pose is not None: model.remove(pose)
        checks['vehicle_unchanged'] = normalized_xml(model) == normalized_xml(vehicle)
        world.find('world').remove(model)
    return {'checks': checks, 'passed': all(checks.values()),
            'world_without_vehicle_sha256': hashlib.sha256(normalized_xml(world)).hexdigest(),
            'vehicle_sha256': hashlib.sha256(normalized_xml(vehicle)).hexdigest()}


def readiness_evidence(record):
    try:
        ready = record['readiness']
        return (graph_ready(ready['graph']) and all(
            ready[t]['rc'] == 0 and topic_connections(ready[t]['stdout'])['publishers'] == count
            and topic_connections(ready[t]['stdout'])['subscribers'] >= 1
            for t, count in EXPECTED_GZ_PUBLISHERS.items()))
    except (KeyError, TypeError):
        return False


def analyze_run(run, contract):
    report = json.loads((run/'report.json').read_text())
    checked = run_result(run, contract, PLUGIN_SHA)
    events = [json.loads(l) for l in (run/'plugin_command_events.jsonl').read_text().splitlines()]
    record = report['evaluation_startup']
    embedded = record['mode'] == 'embedded_paused'
    startup = audit_startup(record, events) if embedded else None
    shape = world_and_model(run, report)
    reader = report.get('startup_reader_audit', {})
    checks = {'run_evidence': checked['valid'], 'gc_policy': checked['checks']['gc_policy'],
              'shape': shape['passed'], 'startup': not embedded or startup['passed'],
              'readiness_evidence': not embedded or readiness_evidence(record),
              'startup_reader': not embedded or (reader.get('handled_messages', 0) >= 2
                  and reader.get('parse_errors') == 0 and reader.get('handler_errors') == 0)}
    start = report['command_schedule']['start_ns']
    end = start+round(report['command_profile']['duration_s']*1e9)+200_000_000
    return {'name': run.name, 'mode': record['mode'], 'checks': checks, 'passed': all(checks.values()),
            'startup': startup, 'shape': shape, 'trace': trace_summary(events, start, end),
            'max_lateness_ms': checked['lateness_ms']['max'], 'source_sha256': checked['source_sha256'],
            'meshes_sha256': checked['meshes_sha256'], 'profile': checked['profile'], 'spawn_pose': checked['spawn_pose'],
            'command_start_ns': start, 'release_wall_s': record.get('release_wall_s'),
            'runtime_vehicle_sha256': checked['runtime_vehicle_sha256'],
            'peak_body_speed_mps': report['peak_body_speed_mps'], 'stopped': report['stopped'],
            'shutdown_clean': report['shutdown_clean']}


def compare(runs):
    embedded = [r for r in runs if r['mode'] == 'embedded_paused']
    baseline = [r for r in runs if r['mode'] == 'service']
    fields = ('source_sha256', 'meshes_sha256', 'profile', 'spawn_pose', 'runtime_vehicle_sha256')
    same = bool(runs) and all(all(r[k] == runs[0][k] for k in fields) and
                             r['shape']['world_without_vehicle_sha256'] == runs[0]['shape']['world_without_vehicle_sha256']
                             for r in runs)
    enough = len(baseline) == 1 and len(embedded) == 2
    matched = enough and embedded[0]['trace']['spawn'] == embedded[1]['trace']['spawn']
    return {'same_except_initialization': same, 'planned_samples_present': enough, 'initial_step_repeat_equal': matched,
            'passed': enough and same and matched and all(r['passed'] for r in runs),
            'application_difference': first_application_difference(embedded[0]['trace'], embedded[1]['trace']) if enough else None,
            'limits': '초기 생성 시점 검증만 한다. ARM/명령 전달·적용의 결정성, 물리 수렴·주행/실차 안전 검증이 아니다.'}


def bounded_confirmation(baseline, corrected, correction):
    """실패 후 남은 한 번의 기능 확인. 원래 두 반복 수용 조건과 분리한다."""
    before, after = baseline['source_sha256'], corrected['source_sha256']
    changed = {k for k in before.keys() | after.keys() if before.get(k) != after.get(k)}
    allowed = correction['approved_implementation_changes']
    source_ok = bool(changed) and changed == set(allowed) and all(
        before.get(k) == allowed[k]['before'] and after.get(k) == allowed[k]['after'] for k in changed)
    fields = ('meshes_sha256', 'profile', 'spawn_pose', 'runtime_vehicle_sha256')
    same = all(baseline[k] == corrected[k] for k in fields) and (
        baseline['shape']['world_without_vehicle_sha256'] == corrected['shape']['world_without_vehicle_sha256'])
    ok = (baseline['mode'] == 'service' and corrected['mode'] == 'embedded_paused'
          and baseline['passed'] and corrected['passed'] and same and source_ok)
    return {'single_corrected_run_confirmed': bool(ok), 'runtime_equal_except_initialization': same,
            'source_difference_exactly_registered_fix': source_ok, 'source_differences': sorted(changed),
            'repeatability_verified': False, 'command_application_determinism_verified': False}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, action='append', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--contract', type=Path, default=ROOT/'config/tests/scheduled_command_r5_sha256.json')
    args = p.parse_args()
    contract = json.loads(args.contract.read_text())
    runs = [analyze_run(run, contract) for run in args.run]
    result = {'runs': runs, 'comparison': compare(runs)}
    with args.output.open('x', encoding='utf-8') as stream: json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps({'runs': [{k:r[k] for k in ('name', 'passed', 'max_lateness_ms', 'release_wall_s')} for r in runs],
                      'comparison': result['comparison']}, ensure_ascii=False, indent=2))
