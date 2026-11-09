"""B 실제 기록(지주 반환 포함)의 저장 입력에서 두 후보 OFF/ON 비교(평가 전용, 열린 고리)."""
import json, sys
from replay_tools import HERE, build_controller, digest, read_events
FLAGS = dict(narrow_occluder_bridge=True, junction_continuity=True)
def main():
    from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze, replay_call
    source = HERE / 'b_solo_posts/follow_decisions.jsonl'; before = digest(source)
    rows = []; runs = None
    for e in read_events(source):
        d = e['data']
        if e['kind'] == 'header':
            runs = {'off': build_controller(d), 'on': build_controller(dict(d, follow_config={**d['follow_config'], **FLAGS}))}
            continue
        if e['kind'] == 'reset':
            for c, _ in runs.values(): c.reset()
            continue
        if e['kind'] != 'call': continue
        out = {k: freeze(replay_call(c, d['input'], mc)) for k, (c, mc) in runs.items()}
        det = out['on'][2] if isinstance(out['on'][2], dict) else {}
        rows.append(dict(t=d['input']['reference_s'], rec=float(d['output'][0]), off=float(out['off'][0]), on=float(out['on'][0]),
                         off_rec=equal_output(comparable_decision(out['off']), comparable_decision(freeze(d['output']))),
                         sel=det.get('boundary_selection'), bridged=det.get('narrow_occluders_bridged'), reason=det.get('reason')))
    assert digest(source) == before
    def zero_windows(key):
        w, cur = [], None
        for r in rows:
            if r[key] <= 1e-6 and r['t'] > 8.4:
                if cur and r['t'] - cur[1] < .05: cur[1] = r['t']
                else: cur = [r['t'], r['t']]; w.append(cur)
        return [[round(a, 2), round(b, 2)] for a, b in w if b - a >= .04]
    s = dict(source_sha256=before, calls=len(rows), off_equals_recorded=sum(r['off_rec'] for r in rows),
             zero_off=sum(r['off'] <= 1e-6 for r in rows if r['t'] > 8.4), zero_on=sum(r['on'] <= 1e-6 for r in rows if r['t'] > 8.4),
             zero_windows_off=zero_windows('off'), zero_windows_on=zero_windows('on'),
             junction_selected=sum(r['sel'] == 'junction_opposite_continuity' for r in rows),
             bridged_calls=sum(bool(r['bridged']) for r in rows),
             post_window_mean_off=sum(r['off'] for r in rows if 9.6 <= r['t'] <= 10.8)/max(1, sum(9.6 <= r['t'] <= 10.8 for r in rows)),
             post_window_mean_on=sum(r['on'] for r in rows if 9.6 <= r['t'] <= 10.8)/max(1, sum(9.6 <= r['t'] <= 10.8 for r in rows)),
             scope='open-loop; recorded B inputs contain real Gazebo post returns; planner only')
    (HERE / 'b_candidate_replay_summary.json').write_text(json.dumps(s, indent=1) + '\n', encoding='utf-8', newline='\n')
    with open(HERE / 'b_candidate_rows.jsonl', 'w', encoding='utf-8', newline='\n') as f:
        for r in rows: f.write(json.dumps(r) + '\n')
    print(json.dumps(s, indent=1))
if __name__ == '__main__': sys.exit(main())
