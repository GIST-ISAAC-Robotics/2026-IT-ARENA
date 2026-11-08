"""evaluation.json·sensitivity.json에서 간결한 근거 요약을 만든다(새 판정 없음)."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ev = json.loads((HERE / 'evaluation.json').read_bytes())
se = json.loads((HERE / 'sensitivity.json').read_bytes())
cr = json.loads((HERE / 'crossing.json').read_bytes())
r3 = lambda x: round(x, 3) if isinstance(x, float) else x
pick = lambda d, ks: None if d is None else {k: r3(d[k]) for k in ks if k in d}
runs = {}
for n, r in ev['runs'].items():
    s = se['runs'][n]
    entry = r['entry_notch_scans']
    occl = [dict(s=x['s'], gap_returns=x['gap_returns'], actor=x['actor_on_sightline'])
            for x in s['raw_gap_returns_by_scan'] if x['actor_on_sightline']]
    runs[n] = dict(
        verdict=r['verdict'], scans=r['scans'], window_scans=r['window_scans'],
        id20=dict(first_status=pick(r['id20']['first_status'], ['t', 's', 'speed_mps']),
                  distinct_camera_stamps_s=r['id20']['distinct_camera_stamps_s'],
                  first_camera=pick(r['id20']['first_camera'], ['t', 's'])),
        C4_left_termination=dict(scans=r['C4_left_termination_scans'],
                                 first=pick(r['C4_left_termination_first'], ['t', 's', 'C4_inner_forward_m'])),
        raw_gap_returns_first_ge3=pick(s['raw_gap_returns_first_ge3_before_E1'], ['t', 's', 'E1_forward_m', 'gap_returns']),
        actor_sightline_occlusion=occl,
        right_termination_at_entry_first=pick(r['right_termination_entry_first'], ['t', 's', 'E1_forward_m']),
        entry_notch_first=dict(pick(r['entry_notch_first'], ['t', 's', 'speed_mps', 'E1_forward_m', 'center_forward_m', 'gap_m']),
                               E1_time_left_const_speed_approx_s=r3(r['entry_notch_first']['E1_time_left_s']),
                               center_time_left_const_speed_approx_s=r3(r['entry_notch_first']['center_time_left_s'])),
        actual_E1_line_crossing=pick(cr['runs'][n], ['t_cross_s', 'actual_time_detection_to_E1_line_s', 'sample_interval_s',
                                                      'x_at_cross_m', 'approx_minus_actual_s']),
        entry_notch_scans_total=len(entry), entry_notch_longest_consecutive_before_E1=r['entry_notch_longest_consecutive'],
        entry_notch_gap_range_m=[r3(min(e['gap_m'] for e in entry)), r3(max(e['gap_m'] for e in entry))] if entry else None,
        entry_notch_endpoint_error_max_m=r3(max(max(e['err_m']) for e in entry)) if entry else None,
        right_notch_by_match=r['right_notch_by_match'], right_notch_false=len(r['right_notch_false']),
        right_notch_in_C4_windows=r['right_notch_in_C4_windows'],
        right_termination_scans_by_place=r['right_termination_scans_by_place'],
        relaxed_45deg_first_entry=pick(s['relaxed_entry_first'], ['t', 's', 'E1_forward_m']),
        relaxed_false=s['relaxed_false_scans'],
        inputs=r['inputs'], observations_sha256=r['observations_sha256'])
out = dict(
    scope=ev['scope'], registered_plan='PLAN.md',
    time_definitions=dict(const_speed_approx='body-frame forward distance to E1 (or entry centre) at detection / instantaneous ego speed; '
                                             'named E1_time_left_s/center_time_left_s in evaluation.json',
                          actual='crossing.json: ego base_link (chassis centre) crosses world y=4.80 near x=6.14, '
                                 'linear interpolation between adjacent ego_poses samples, minus detection reference time'),
    observer_sha256=hashlib.sha256((HERE / 'observe.py').read_bytes()).hexdigest(),
    feature='right-side collinear wall break: wall A ends (E1), 0.40-0.85 m gap with >=3 real returns >=0.25 m beyond the line, collinear wall C resumes (E2)',
    world_fact='approach view: right wall x=5.635 ends at y=4.80; through the 0.60 m gap the island north face (y=4.20) is seen; it meets the collinear island east face at E2. The P22 north wall face is not visible from the approach.',
    runs=runs,
    history=['first evaluate.py pass reported 0 right notches/verdict 판별 불가 because notch items were labelled '
             '"notche" (string bug); observations were unchanged, only the label was fixed and evaluation rerun.',
             'first figure render used Korean labels missing from the WSL matplotlib font; labels changed to English.',
             'sensitivity.py added after the registered verdict to separate threshold vs visibility; it does not change the verdict.',
             'after final review, crossing.py added actual E1-line crossing times; the earlier time-left values are kept and renamed as constant-speed approximations.'],
    reproduction=dict(command='bash run.sh (WSL, ROS jazzy environment)', first_pass_sha256='first_pass.sha256',
                      rerun_matched=True))
(HERE / 'summary.json').write_bytes((json.dumps(out, ensure_ascii=False, indent=1) + '\n').encode('utf-8'))
print(json.dumps({n: {k: v for k, v in r.items() if k != 'inputs'} for n, r in runs.items()}, ensure_ascii=False, indent=1))
