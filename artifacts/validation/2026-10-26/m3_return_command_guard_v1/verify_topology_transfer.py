"""원문 계약 시제품을 동일 합성 관문에 대조. 실시간/운영 연결이 아닌 고정 입력 시험."""
from collections import OrderedDict
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'m3_rear_residual_v1'
sys.path.insert(0,str(OLD))
import counterexamples as cx
import release_premises as rp
from types import SimpleNamespace
from arena_autonomy.scan_topology import scene_with_cycle
from arena_autonomy.observation_support import depth_support as source_depth
from arena_autonomy.m3_execution_trial import _both_sides_returned as source_returned
sc = SimpleNamespace(scene_with_cycle=scene_with_cycle, cycle_depth_support=source_depth,
                     cycle_both_returned=source_returned)
import arena_autonomy.m3_execution_trial as mx


def run(case,half_angle,enabled):
    original=cx.world_for
    contexts=OrderedDict()
    def with_cycle(name):
        w=original(name)
        def scene(t,bodies=None):
            ranges,stamps=w.ranges(t,bodies)
            context=sc.scene_with_cycle(ranges.tolist(),angle_min=-math.pi,angle_increment=2*math.pi/500,
                range_min=.05,range_max=12.,acquired_at_s=stamps.tolist(),t_ref_s=t,
                acquisition_to_ref_se2=[(w.ego_x(s)-w.ego_x(t),0.,0.) for s in stamps],rear_half_angle_deg=half_angle)
            contexts[id(context.scene)]=context
            # Current, previous aggregate and pending scenes are retained; no pointer-only cache.
            while len(contexts)>16:contexts.popitem(last=False)
            return context.scene
        w.scene=scene
        return w
    def proof(scene):
        c=contexts.get(id(scene))
        return c if enabled and c is not None and c.scene is scene else None
    def depth(scene,queries,padding):return sc.cycle_depth_support(scene,queries,padding,cycle=proof(scene))
    def returned(scene,queries):return sc.cycle_both_returned(scene,queries,cycle=proof(scene))
    saved=(rp.depth_support,mx.depth_support,mx._both_sides_returned)
    cx.world_for=with_cycle
    rp.depth_support,mx.depth_support,mx._both_sides_returned=depth,depth,returned
    try:
        return cx.run((case,'union_strict'))[2]
    finally:
        cx.world_for=original
        rp.depth_support,mx.depth_support,mx._both_sides_returned=saved


def main(output,baseline,seam_baseline):
    if output.exists():raise FileExistsError(output)
    raw=json.loads(baseline.read_text());seam=json.loads(seam_baseline.read_text())
    files=[Path(__file__),Path(__file__).parents[4]/'src/arena_autonomy/arena_autonomy/scan_topology.py',OLD/'release_premises.py',OLD/'synthetic_world.py',Path(mx.__file__)]
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    results={};checks={}
    for case in ['left_pass','right_pass','same_side_run37','reapproach']:
        for half in [30.,0.]:
            key=f'{case}__union_strict__{half:g}'
            for enabled in [False,True]:
                rows=run(case,half,enabled);name=key+('__on' if enabled else '__off')
                expected=seam[key] if enabled else raw[key]
                checks[name+'_matches_independent_reference']=rows==expected
                results[name]={'summary':cx.summarize(rows),'rows':rows}
                print(json.dumps({name:{'matches_reference':rows==expected,'release':results[name]['summary']['released_at']}},ensure_ascii=False),flush=True)
    unchanged=all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p)] for p in files)
    report={'scope':'integrated opt-in API vs previous fixture reference, open-loop only',
        'checks':checks,'source_sha256':hashes,'source_unchanged':unchanged,
        'results':{k:v['summary'] for k,v in results.items()},
        'limits':['No ROS/Gazebo/real C1 or operational integration.',
                  'Full-turn proof describes raw array topology, not timestamp accuracy.',
                  'No P-follow or P-size exemption; current rear mask remains unchanged.']}
    output.mkdir(parents=True)
    (output/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (output/'rows.json').write_text(json.dumps({k:v['rows'] for k,v in results.items()})+'\n',encoding='utf-8')
    return 0 if unchanged and all(checks.values()) else 1


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--seam-baseline',type=Path,required=True)
    a=p.parse_args();sys.exit(main(a.output,a.baseline,a.seam_baseline))
