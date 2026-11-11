"""기존 판단 로그/취득 매칭 판정에서 명시 native 입력의 첫 점 시각을 읽는다."""
import json, statistics
read=lambda p:json.loads(p.read_bytes())
jsonl=lambda p:[json.loads(l) for l in p.read_text(encoding='utf-8').splitlines() if l.strip()]

def native_first_stamp(scan):
    if scan.get('representation') != 'native_measurements_v1':
        raise ValueError('expected native decision input')
    values=scan['measurements']
    if values['time_basis'] != 'simulation_acquisition':
        raise ValueError('unexpected native time basis')
    return values['acquired_at_s'][0]

def decision_gates(run, acquisition):
    rows = jsonl(run/'follow_decisions.jsonl')
    footer = [row['data'] for row in rows if row['kind'] == 'footer']
    calls = [row['data']['input'] for row in rows if row['kind'] == 'call']
    window = read(run/'report.json')['case_window']
    all_calls = calls
    calls = [c for c in calls if window['green_sim_s'] <= c['reference_s'] <= window['end_sim_s']]
    stamps = {round(row['first_ray_stamp_s'], 6) for row in acquisition}
    ages, missing, scans = [], 0, set()
    for call in calls:
        stamp = round(native_first_stamp(call['scan']), 6)
        scans.add(stamp); missing += stamp not in stamps
        ages.append(call['reference_s']-stamp)
    gates = dict(decision_scans_acquired=bool(calls) and missing == 0,
                 decision_log_complete=len(footer) == 1 and footer[0].get('dropped') == 0 and footer[0].get('failed') == 0
                     and footer[0].get('capture_errors') == 0 and footer[0].get('accepted') == footer[0].get('written'))
    q = sorted(ages)
    age = dict(calls=len(calls), unique_scans=len(scans), not_in_acquisition=missing,
               min=q[0] if q else None, p50=statistics.median(q) if q else None,
               p95=q[min(len(q)-1, int(.95*len(q)))] if q else None, max=q[-1] if q else None,
               scope='case-window decision reference_s minus first-ray stamp; footer completeness covers all calls',
               excluded_outside_case_calls=len(all_calls)-len(calls),
               all_calls_not_in_acquisition=sum(round(native_first_stamp(c['scan']),6) not in stamps for c in all_calls))
    return gates, age
