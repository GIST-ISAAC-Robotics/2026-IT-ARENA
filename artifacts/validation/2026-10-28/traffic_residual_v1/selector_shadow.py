"""통과 이력·동일 경로 좁은 결측의 저장 입력 비교 사본. 운영 연결 없음."""
import importlib.util
from pathlib import Path
import sys
import types
from probe_same_path_gap import candidate

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]

def selector_class():
    path=ROOT/'artifacts/validation/2026-10-27/obstacle_latch_v1/probe_pass_continuity.py'
    spec=importlib.util.spec_from_file_location('prior_pass_shadow',path)
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    _,original,source=old.shadow_class()
    func,gap_source=candidate()
    helper='''
def same_path_risk(owner, scene, raw_scan, region, speed, target, steer, previous, dt, baseline):
    rejected={'accepted':False,'reason':'not_needed','unknown_filled':False}
    if not region['unobserved'] or raw_scan is None:
        return rejected
    _,queries=motion_queries(scene,speed,target,steer,previous,dt,owner.follow_config,owner.lat)
    pad=mx.ray_padding(scene,owner.lat,.5)
    free=depth_support(scene,queries,pad)
    masked=np.zeros(len(queries),dtype=bool)
    masked[~free]=mx.masked_only(scene,queries[~free],pad)
    proof=baseline.get('partial_observation') or {}
    command=dict(now=scene.t_ref_s,speed=speed,target=target,steering=steer,previous=previous)
    return same_path_gap(scene,raw_scan,queries[~free & ~masked],pad,
        owner.follow_config.length_m,owner.follow_config.width_m,proof,command)

'''
    anchor='class PassReleaseSelector:'
    assert source.count(anchor)==1;source=source.replace(anchor,helper+anchor)
    old="        info['rear_fringe'] = fringe"
    new="""        if not fringe['accepted']:
            same = same_path_risk(self,scene,raw_scan,region,speed,target,b_steer,previous,dt,b_detail)
            info['same_path_gap'] = same
            if same['accepted']:fringe=same
        info['rear_fringe'] = fringe"""
    assert source.count(old)==1;source=source.replace(old,new)
    old="""        detail['selector_region'] = region
        blocked = region_blocked(region, self.config)"""
    new="""        detail['selector_region'] = region
        same=same_path_risk(self,scene,raw_scan,region,speed,b_speed,b_steer,previous,dt,b_detail)
        detail['same_path_gap']=same
        blocked = region_blocked(region, self.config, bounded_unknown_risk=same['accepted'])"""
    assert source.count(old)==1;source=source.replace(old,new)
    module=types.ModuleType('combined_pass_shadow');module.__file__=str(Path(__file__))
    sys.modules[module.__name__]=module;module.same_path_gap=func
    exec(compile(source,str(Path(__file__)),'exec'),module.__dict__)
    return module.PassReleaseSelector,original,source,gap_source
