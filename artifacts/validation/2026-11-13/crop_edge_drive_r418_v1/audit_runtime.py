"""새 runtime trace의 선택·후보 계산·부모 발행을 구분한다. 원 주행 판정을 바꾸지 않는다."""
import argparse
from collections import Counter
import json
from pathlib import Path
from header_comparison import compare_headers

KEY = 'current_corner_distance_budget'


def component_summary(rows):
    paths = Counter(); allowed = Counter(); selected_calls = positive = 0
    def walk(value, path):
        if isinstance(value, dict):
            partial = value.get('partial_observation')
            if isinstance(partial, dict) and isinstance(partial.get('crop_edge_component'), dict):
                yield path+'.partial_observation.crop_edge_component', partial
            for key, item in value.items():
                yield from walk(item, path+'.'+key)
        elif isinstance(value, list):
            for index, item in enumerate(value):
                yield from walk(item, path+'['+str(index)+']')
    for row in rows:
        if row['kind'] != 'call': continue
        output = row['data'].get('output')
        entries = list(walk(output, 'output'))
        if not entries: continue
        selected_calls += 1
        positive += int(isinstance(output, list) and len(output) == 3
                        and type(output[0]) in (int, float) and output[0] > 0)
        for path, partial in entries:
            paths[path] += 1
            allowed[str(partial.get('allowed', 'missing'))] += 1
    return dict(plan_call_outputs_with_diagnostic=selected_calls,
        positive_plan_call_outputs_with_diagnostic=positive, diagnostic_paths=dict(paths),
        partial_observation_allowed=dict(allowed),
        diagnostic_is_not_guard_or_mcu_application=True)


def summarize(rows, expected):
    headers = [r['data'] for r in rows if r['kind'] == 'header']
    factory = [r['data'] for r in rows if r['kind'] == 'runtime_factory_policy']
    submission = [r['data'] for r in rows if r['kind'] == 'decision_submission_policy']
    calls = [r['data'] for r in rows if r['kind'] == 'call']
    reasons = Counter(); tried = accepted = published = 0
    pending = None
    for row in rows:
        data = row['data']
        if row['kind'] == 'call':
            current = data.get('current_rebase', {}).get('output', {})
            diagnostic = current.get(KEY)
            pending = None
            if diagnostic:
                tried += 1; reasons[diagnostic.get('reason', 'missing')] += 1
                if diagnostic.get('applied') is True:
                    accepted += 1
                    if data.get('delivery', {}).get('kind') == 'candidate':
                        pending = current
        elif row['kind'] == 'route_publication':
            if (pending is not None and data.get('retained') is True
                    and data.get('reference_s') == pending['reference_s']
                    and data.get('steering_rad') == pending['steering_rad']
                    and 0 < data.get('speed_mps', 0) <= pending['speed_mps']):
                published += 1
            pending = None
        elif row['kind'] == 'reset':
            pending = None
    footer = rows[-1]['data'] if rows and rows[-1]['kind'] == 'footer' else None
    comparison = compare_headers(headers[0], expected) if len(headers) == 1 else dict(
        equivalent=False, exact=False, error='header count is not one')
    common_identity = (len(factory) == 1
        and factory[0].get('policy') == 'cad_runtime_r377_v1'
        and factory[0].get('current_corner_budget') == 'r396_v1'
        and factory[0].get('motion_time_policy') == 'verified_only'
        and factory[0].get('motion_timing_verified') is True
        and len(submission) == 1 and submission[0].get('policy') == 'runtime_dual_latest_v1')
    identity = comparison['exact'] and common_identity
    equivalent = comparison['equivalent'] and common_identity
    return dict(configuration_evidence_valid=identity, header_exact=comparison['exact'],
        header_comparison=comparison, configuration_equivalent=equivalent,
        crop_edge_component_selected=(len(headers) == 1 and
            headers[0].get('follow_config', {}).get('crop_edge_component_trial') is True),
        crop_edge_diagnostics=component_summary(rows),
        calls=len(calls), candidate_diagnostics=tried, candidate_calculations_accepted=accepted,
        candidate_parent_publications=published, candidate_reasons=dict(reasons),
        trace_closed=footer is not None, capture_errors=None if footer is None else footer['capture_errors'],
        parent_publications_observed=published,
        parent_publication_evidence_valid=bool(equivalent and footer is not None
            and footer.get('capture_errors') == 0 and published > 0),
        driving_improvement_demonstrated=False,
        independent_guard_or_mcu_application_proven_by_this_audit=False,
        original_driving_verdict_unchanged=True,
        scope='parent checked-route publication only; original run report owns STOP/contact/surface/progress verdict')


def audit(trace, expected):
    rows = [json.loads(line) for line in trace.read_text(encoding='utf8').splitlines()]
    return summarize(rows, json.loads(expected.read_bytes()))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('trace', type=Path); p.add_argument('expected', type=Path)
    args = p.parse_args(); print(json.dumps(audit(args.trace, args.expected), ensure_ascii=False, indent=2))
