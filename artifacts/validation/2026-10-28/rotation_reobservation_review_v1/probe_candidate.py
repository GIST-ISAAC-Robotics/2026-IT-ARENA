"""복수 실패 입력의 분리된 재관측 상태 비교. 새 주행/전체 제어 재생 아님."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,thaw,freeze
from arena_autonomy.lidar_motion import MotionConfig
import arena_autonomy.surface_anchor_trial as anchor_module
from extract_latch import digest


def main(fixture,output,variant='small'):
    import importlib
    module=importlib.import_module(dict(fixed='consistent_alignment_candidate',
        finite='finite_segment_candidate',small='small_alignment_candidate')[variant])
    candidate=module.surface_translation
    if output.exists():raise FileExistsError(output)
    before=digest(fixture)
    data=thaw(json.loads(fixture.read_text(encoding='utf-8')))
    e=data['evidence'];calls=data['calls'];motion=MotionConfig(**data['header']['motion_config'])
    original=next(r for r in calls if abs(r['input']['reference_s']-e['previous_reference_s'])<1e-8)
    inp,scan,history=restore_call(original['input'],motion)
    old,ids,_=history.project(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,
        scan.range_max,e['previous_source_first_s'],scan.time_increment,inp['reference_s'],'both',True)
    keep=np.linalg.norm(old,axis=1)<=3.;old,ids=old[keep],ids[keep]
    ctx=e['correspondence_context'];np.testing.assert_allclose(old,ctx['previous_points_at_reference_xy'],atol=1e-8,rtol=0)
    retained=set();moved=np.asarray(ctx['previous_points_now_xy'])
    for witness in e['witnesses']:
        dist=np.linalg.norm(moved-witness['previous_point_now_xy'],axis=1)
        index=int(np.argmin(dist));assert dist[index]<1e-8;retained.add(index)
    probes={name:anchor_module.SurfaceAnchorTrial(extended_surface=True) for name in ('baseline','candidate')}
    for probe in probes.values():probe.start(old[sorted(retained)],old,ids,e['previous_reference_s'],e['previous_source_first_s'])
    original_fit=anchor_module.surface_translation
    rows=[];last=None;first_match={k:None for k in probes}
    try:
        for event in calls:
            inp,scan,history=restore_call(event['input'],motion);now=inp['reference_s']
            if now<e['reference_s']:continue
            source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            current,ids,_=history.project(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,
                scan.range_max,source,scan.time_increment,now,'both',True)
            values={}
            for name,probe in probes.items():
                anchor_module.surface_translation=original_fit if name=='baseline' else candidate
                values[name]=probe.observe(current,ids,history,now,source)
                if values[name]['matched'] and first_match[name] is None:first_match[name]=now
                if 'fit' in values[name]:values[name]['fit'].pop('processing_ms',None)
            if source!=last:
                rows.append(dict(t=now,**values));last=source
    finally:
        anchor_module.surface_translation=original_fit
    output.write_text(json.dumps(freeze(dict(scope='isolated probe; no control or safety release',
        candidate_variant=variant,candidate_source_sha256=digest(Path(module.__file__)),
        input_sha256=before,input_unchanged=digest(fixture)==before,first_match_s=first_match,rows=rows)),
        ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(output=str(output),first_match_s=first_match,input_unchanged=digest(fixture)==before)))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('fixture',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--variant',choices=('small','fixed','finite'),default='small');args=parser.parse_args()
    main(args.fixture,args.output,args.variant)
