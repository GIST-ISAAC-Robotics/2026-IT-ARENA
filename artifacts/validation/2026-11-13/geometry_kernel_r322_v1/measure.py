"""r296 고정106입력에서 투영 구현만 교대 비교. 실제 ROS/동시 부하 아님."""
from pathlib import Path
import argparse,hashlib,importlib.util,json,platform,statistics,time
A=Path(__file__).resolve().parent;R=A.parents[3];B=A.parent/'current_input_jetson_r296_v1'
spec=importlib.util.spec_from_file_location('cost_reference',B/'measure_cost.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
from arena_autonomy import native_geometry,numeric_kernels


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--rounds',type=int,default=3);args=parser.parse_args()
    args.out.mkdir(exist_ok=False)
    manifest=json.loads((A/'source_manifest.json').read_bytes())
    def changed():return [p for p,h in manifest.items() if hashlib.sha256((R/p).read_bytes()).hexdigest()!=h]
    assert not changed(),changed()
    old.configure('native_v1');assert hasattr(numeric_kernels._backend,'geometry')
    original=native_geometry.numeric_geometry
    headers=json.loads((B/'headers.json').read_bytes())
    configs={k:(old.MotionConfig(**v['motion_config']),old.FollowConfig(**v['follow_config'])) for k,v in headers.items()}
    entries=json.loads((B/'selected_inputs.json').read_bytes())
    def evaluate(entry):
        c=entry['input'];motion,cfg=configs[entry['source']];current=c['current_rebase']['input']
        d,scan,history=old.restore_call(current,motion)
        scene=old.motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        return old.freeze(old.candidate.rebase(c['output'],old.plan_reference(c['input']),current,scene,history,cfg,require_current_region=True))
    rows=[];answers={};full=operational=pair=exact=0
    fields=('speed_mps','steering_rad','reason','reference_s','original_reference_s','source_first_s','path_source_s','independent_guard_required','runtime_authorized')
    for round_ in range(args.rounds):
        order=('original','candidate') if round_%2==0 else ('candidate','original')
        round_answers={}
        for mode in order:
            native_geometry.numeric_geometry=(lambda *a:None) if mode=='original' else original
            evaluate(entries[0])
            values=[]
            for entry in entries:
                start=time.perf_counter_ns();out=evaluate(entry);elapsed=(time.perf_counter_ns()-start)/1e6
                rows.append(dict(round=round_,mode=mode,source=entry['source'],index=entry['index'],elapsed_ms=elapsed))
                values.append(out)
                expected=entry['output']
                full+=int(old.equal_output(out,expected))
                operational+=int(old.equal_output({k:out.get(k) for k in fields},{k:expected.get(k) for k in fields}))
            round_answers[mode]=values
        for before,after in zip(round_answers['original'],round_answers['candidate']):
            pair+=int(old.equal_output(before,after))
            exact+=int(json.dumps(before,sort_keys=True)==json.dumps(after,sort_keys=True))
        answers=round_answers
    native_geometry.numeric_geometry=original
    def stats(values):
        v=sorted(values);return dict(n=len(v),median=statistics.median(v),p95=v[int((len(v)-1)*.95)],maximum=v[-1])
    groups={label:{mode:stats([r['elapsed_ms'] for r in rows if r['source']==label and r['mode']==mode])
                   for mode in ('original','candidate')} for label in headers}
    report=dict(platform=platform.platform(),groups=groups,rounds=args.rounds,entries=len(entries),
        full_reference_equal=full,operational_equal=operational,pair_equal=pair,pair_exact_json=exact,
        changed_sources=changed(),selected_sha256=hashlib.sha256((B/'selected_inputs.json').read_bytes()).hexdigest(),
        scope='fixed input restore + scene + current recheck + freeze; not ROS or concurrent latency bound')
    for name,value in [('report',report),('durations',rows),('answers',answers)]:
        (args.out/(name+'.json')).write_text(json.dumps(value,separators=(',',':'))+'\n')
    print(json.dumps(report));assert not report['changed_sources']
    assert pair==args.rounds*len(entries) and operational==2*args.rounds*len(entries)


if __name__=='__main__':main()
