"""검증된 합성 1회전의 끝/첫 광선 인접성 비교. 운영 코드에는 연결하지 않는다.

합성 생성기의 취득 시각·원시 각도·순서와 일치하는 완전 500광선에만 순환 인접성을 허용한다.
실물 원문 metadata가 없는 LocalScene에 이 전제를 자동 추정해서 채택하지 않는다.
양옆 광선 상태/각도/점별 여유·거리/가능 영역·해제 조건은 원래 함수를 재사용한다.
"""
import argparse
from dataclasses import replace
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import numpy as np
import counterexamples as cx
import release_premises as rp
import review_visibility as rv
import arena_autonomy.observation_support as osupport
import arena_autonomy.m3_execution_trial as mx


def synthetic_cycle_verified(scene):
    """Exact fixture contract: translation only, one ordered turn over 0.1 s."""
    if len(scene.rays)!=500: return False
    for i,r in enumerate(scene.rays):
        a=-math.pi+i*2*math.pi/500
        if (r.source_index!=i or r.source_epoch!=scene.source_epoch or
            r.source_frame_id!=scene.source_frame_id or
            abs(r.acquired_at_s-(scene.t_ref_s-.1+i*.0002))>1e-9 or
            abs(r.direction_xy[0]-math.cos(a))>1e-9 or
            abs(r.direction_xy[1]-math.sin(a))>1e-9): return False
    return True


def clone_with_cycle(fn, expression):
    source=inspect.getsource(fn)
    if source.count(expression)!=1: raise ValueError('source expression no longer unique')
    replacement='('+expression+' | ((np.abs(indices[lo]-indices[hi]) == len(indices)-1) & synthetic_cycle_verified(scene)))'
    source=source.replace(expression,replacement)
    namespace=dict(fn.__globals__)
    namespace['synthetic_cycle_verified']=synthetic_cycle_verified
    exec(compile(source,'<offline synthetic-cycle comparison>','exec'),namespace)
    return namespace[fn.__name__]


def fixture_checks(depth):
    world=cx.sw.World(0.,0.)
    ranges,stamps=world.ranges(1.)
    scene=rv.scene_with_mask(world,1.,ranges,stamps,0.)
    a=-math.pi+.5*(2*math.pi/500) # Neighbor 0/1; use other side for 499/0 seam.
    def query(bearing): return np.array([[.06+math.cos(bearing),math.sin(bearing)]])
    seam=query(math.pi-.5*(2*math.pi/500)); adjacent=query(a)
    pad=np.zeros(500)
    def changed(index,**kw):
        rays=list(scene.rays); rays[index]=replace(rays[index],**kw)
        return replace(scene,rays=tuple(rays),source_first_s=rays[0].acquired_at_s,
                       source_last_s=rays[-1].acquired_at_s)
    missing=replace(scene,rays=scene.rays[:-1],source_last_s=scene.rays[-2].acquired_at_s)
    stale=changed(0,acquired_at_s=0.0)
    largepad=pad.copy(); largepad[0]=10.
    unknown=changed(0,status='unknown',hit_xy_m=None)
    masked=changed(499,status='masked',hit_xy_m=None)
    closehit=changed(0,hit_xy_m=(.06-.1,0.))
    checks={
        'complete_fixture_metadata_verified':synthetic_cycle_verified(scene),
        'baseline_seam_rejected':not bool(osupport.depth_support(scene,seam,pad)[0]),
        'certified_seam_supported':bool(depth(scene,seam,pad)[0]),
        'nonseam_same':bool(depth(scene,adjacent,pad)[0])==bool(osupport.depth_support(scene,adjacent,pad)[0]),
        'missing_last_ray_not_wrapped':not synthetic_cycle_verified(missing) and not bool(depth(missing,seam,np.zeros(499))[0]),
        'wrong_acquisition_time_not_wrapped':not synthetic_cycle_verified(stale) and not bool(depth(stale,seam,pad)[0]),
        'first_unknown_rejected':not bool(depth(unknown,seam,pad)[0]),
        'last_masked_rejected':not bool(depth(masked,seam,pad)[0]),
        'older_ray_padding_preserved':not bool(depth(scene,seam,largepad)[0]),
        'near_return_preserved':not bool(depth(closehit,seam,pad)[0]),
    }
    return checks


def main(output,baseline):
    if output.exists():raise FileExistsError(output)
    base=json.loads(baseline.read_text())
    depth=clone_with_cycle(osupport.depth_support,'(abs(indices[lo]-indices[hi]) <= 1)')
    returned=clone_with_cycle(mx._both_sides_returned,'(np.abs(indices[lo]-indices[hi]) <= 1)')
    checks=fixture_checks(depth)
    paths=[Path(__file__),HERE/'release_premises.py',HERE/'review_visibility.py',HERE/'synthetic_world.py',
           Path(mx.__file__),Path(osupport.__file__)]
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    results={}
    for case in ['left_pass','right_pass','same_side_run37','reapproach']:
        for half in [30.,0.]:
            saved=(rp.depth_support,mx.depth_support,mx._both_sides_returned)
            rp.depth_support,mx.depth_support,mx._both_sides_returned=depth,depth,returned
            try:
                key,summary,rows=rv.job((case,'union_strict',half))
            finally:
                rp.depth_support,mx.depth_support,mx._both_sides_returned=saved
            previous=base[key]
            same=rows==previous
            if half==30.:checks[case+'_current_mask_unchanged']=same
            if case in ('same_side_run37','reapproach'):
                checks[case+f'_{half:g}_hidden_threat_not_released']=not summary['released_at']
            results[key]={'summary':summary,'same_rows_as_run11':same,'rows':rows}
            print(json.dumps({key:{'summary':summary,'same_rows_as_run11':same}},ensure_ascii=False),flush=True)
    unchanged=all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p)] for p in paths)
    report={'scope':'unconnected synthetic cyclic-adjacency comparison; no minimum-size or rear-risk exemptions',
            'checks':checks,'source_sha256':hashes,'source_unchanged':unchanged,
            'results':{k:{a:b for a,b in v.items() if a!='rows'} for k,v in results.items()},
            'limits':['Synthetic ordered full-turn metadata only; not a production scanner metadata implementation.',
                      'Per-ray age padding kept; circular array adjacency does not imply equal measurement times.',
                      'Union state remains an offline prototype. No physical or closed-loop validation.']}
    output.mkdir(parents=True)
    (output/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (output/'rows.json').write_text(json.dumps({k:v['rows'] for k,v in results.items()})+'\n',encoding='utf-8')
    print(json.dumps({'checks':checks,'source_unchanged':unchanged}),flush=True)
    return 0 if all(checks.values()) and unchanged else 1


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True)
    a=p.parse_args();sys.exit(main(a.output,a.baseline))
