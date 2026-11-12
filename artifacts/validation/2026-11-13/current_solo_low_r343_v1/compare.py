"""r338 전체 현재 검사와 이전 교통 대표의 원/후 대조. 주행 미연결."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
spec = importlib.util.spec_from_file_location('r296_compare', A.parent/'current_input_jetson_r296_v1/measure_cost.py')
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)
spec = importlib.util.spec_from_file_location('r343_solo_low', A/'candidate.py')
new = importlib.util.module_from_spec(spec)
spec.loader.exec_module(new)


def main():
    assert not (A/'report.json').exists()
    old.configure('native_v1')
    source = A.parent/'published_route_drive_r338_v1/run1/follow_decisions.jsonl'
    with source.open() as stream:
        header = json.loads(next(stream))['data']
        records = [('r338', header, e['data']) for e in map(json.loads, stream)
                   if e['kind']=='call' and 'current_rebase' in e['data']]
    holdout = A.parent/'current_input_jetson_r296_v1'
    headers = json.loads((holdout/'headers.json').read_bytes())
    records += [('holdout_'+e['source'], headers[e['source']], e['input'])
                for e in json.loads((holdout/'selected_inputs.json').read_bytes())]
    rows, answers = [], []
    configs = {}
    for index, (label, h, c) in enumerate(records):
        if label not in configs:
            configs[label] = (old.MotionConfig(**h['motion_config']), old.FollowConfig(**h['follow_config']))
        motion, cfg = configs[label]
        current = c['current_rebase']['input']
        d, scan, history = old.restore_call(current, motion)
        scene = old.motion_scene(scan, history, d['reference_s'], timing_verified=d['timing_verified'],
                                epoch=d['epoch'], rear_half_angle_deg=d['rear_half_angle_deg'])
        outputs, costs = {}, {}
        order = [('before',old.candidate),('after',new)]
        if index%2: order.reverse()
        for mode, module in order:
            start = time.perf_counter_ns()
            outputs[mode] = old.freeze(module.rebase(c['output'], old.plan_reference(c['input']), current,
                                                      scene, history, cfg, require_current_region=True))
            costs[mode] = (time.perf_counter_ns()-start)/1e6
        before, after = outputs['before'], outputs['after']
        changed = not old.equal_output(before, after)
        retry = after.get('current_solo_low_retry', {})
        if retry.get('applied'):
            assert before['speed_mps'] == 0 and 0 < after['speed_mps'] <= .15
            assert after['reference_s'] == before['reference_s'] and after['steering_rad'] == before['steering_rad']
            assert after['current_route_proof']['accepted']
        elif changed:
            assert old.equal_output(before, {k:v for k,v in after.items() if k!='current_solo_low_retry'})
        rows.append(dict(label=label,index=index,reference_s=d['reference_s'], changed=changed,
            applied=retry.get('applied',False), retry=bool(retry), before_speed=before['speed_mps'],
            after_speed=after['speed_mps'], before_reason=before['reason'],after_reason=after['reason'],
            times_ms=costs))
        answers.append(outputs)
    summary = {label:dict(count=sum(r['label']==label for r in rows),
        changed=sum(r['changed'] for r in rows if r['label']==label),
        applied=sum(r['applied'] for r in rows if r['label']==label)) for label in sorted(configs)}
    report = dict(scope='stored input replay only; default and hardware unchanged', summary=summary, rows=rows,
        hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in (source,holdout/'headers.json',holdout/'selected_inputs.json',Path(__file__),A/'candidate.py')})
    (A/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    (A/'answers.json').write_text(json.dumps(answers, separators=(',',':'))+'\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
