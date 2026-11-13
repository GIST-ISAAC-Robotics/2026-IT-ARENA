"""새 ARM 커널의 경계, 기존 helper 반례 및 실제 저장 배열을 exact로 대조한다."""
import ast
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'stage/src/arena_autonomy'),str(ROOT/'stage/src/arena_vehicle_interface')]
import numpy as np
from arena_autonomy import numeric_kernels as backend,known_structure as candidate,pass_release_trial as ps
from arena_autonomy.follow_trace import restore_call,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene


def main():
    backend.configure('native_v1')
    extension=backend._backend
    assert hasattr(extension,'current_mask_occupied')
    spec=importlib.util.spec_from_file_location('r470_reference',ROOT/'base/src/arena_autonomy/arena_autonomy/known_structure.py')
    original=importlib.util.module_from_spec(spec);spec.loader.exec_module(original)
    kernel_checks=[]
    def check(label,q,h,pad):
        q,h,pad=[np.asarray(v,dtype=float) for v in (q,h,pad)]
        before=[v.copy() for v in (q,h,pad)]
        with np.errstate(all='ignore'):
            expected=any(len(h) and np.any(np.linalg.norm(q[start:start+64,None]-h[None],axis=2)<=pad)
                for start in range(0,len(q),64))
            actual=backend.current_mask_occupied(q,h,pad)
        assert actual==bool(expected),(label,expected,actual)
        assert all(np.array_equal(a,b,equal_nan=True) for a,b in zip((q,h,pad),before))
        kernel_checks.append(dict(label=label,exact=True,result=actual,queries=len(q),hits=len(h)))
    check('empty_queries',np.empty((0,2)),[[0,0]],[.05])
    check('empty_hits',[[0,0]],np.empty((0,2)),[])
    for distance in (.05,5.,math.sqrt(2.)):
        point=[distance,0.] if distance!=math.sqrt(2.) else [1.,1.]
        for label,pad in (('below',np.nextafter(distance,-math.inf)),('equal',distance),('above',np.nextafter(distance,math.inf))):
            check(f'boundary_{distance}_{label}',[point],[[0,0]],[pad])
    for label,q,h,pad in (
        ('diagonal_exact',[[3.,4.]],[[0.,0.]],[5.]),
        ('signed_zero',[[-0.,0.]],[[0.,-0.]],[0.]),
        ('subnormal_square_underflow',[[np.nextafter(0.,1.),0.]],[[0.,0.]],[0.]),
        ('overflow_distance',[[1e308,1e308]],[[0.,0.]],[1e308]),
        ('nan_query',[[math.nan,0.]],[[0.,0.]],[1.]),
        ('nan_padding',[[0.,0.]],[[0.,0.]],[math.nan]),
        ('infinity_padding',[[1e308,1e308]],[[0.,0.]],[math.inf]),
        ('mixed_hit_order',[[1.,1.],[0.,0.]],[[10.,10.],[0.,0.]],[0.,0.])):
        check(label,q,h,pad)
    for seed in (19,73):
        rng=np.random.default_rng(seed)
        q=rng.uniform(-2,2,(97,2));h=rng.uniform(3,5,(83,2));pad=rng.uniform(0,.2,83)
        check(f'random_absent_{seed}',q,h,pad)
        h[-1]=q[-1]
        check(f'random_last_pair_{seed}',q,h,pad)
    raw_checks=[]
    for invalid in ('type','length','readonly','noncontiguous','oversize','negative'):
        q=np.zeros((2,2));h=np.zeros((3,2));pad=np.zeros(3);out=np.empty(1,dtype=np.uint8)
        if invalid=='type':q=q.astype(np.float32)
        elif invalid=='length':pad=np.zeros(2)
        elif invalid=='readonly':out.flags.writeable=False
        elif invalid=='noncontiguous':q=np.zeros((2,4))[:,::2]
        n=1000001 if invalid=='oversize' else -1 if invalid=='negative' else 2
        try:extension.current_mask_occupied(n,3,q,h,pad,out)
        except (ValueError,BufferError):raw_checks.append(dict(case=invalid,rejected=True))
        else:raise AssertionError(('raw buffer accepted',invalid))
    backend.configure('python');assert backend.current_mask_occupied(None,None,None) is None
    backend._backend=object()
    try:backend.current_mask_occupied([],[],[])
    except RuntimeError:missing_rejected=True
    else:raise AssertionError('missing native entrypoint not rejected')
    backend.configure('native_v1')
    helper_checks=[]
    def helper_compare(label,args):
        before=original.current_mask_shadow(*args);after=candidate.current_mask_shadow(*args)
        helper_checks.append(dict(label=label,exact=before==after,output=after))
        assert before==after,(label,before,after)
        return after
    data=json.loads((ROOT/'inputs.json').read_bytes())
    for record in data['records']:
        header=data['headers'][record['dataset']]
        cfg=FollowConfig(**header['follow_config'])
        info,scan,history=restore_call(record['current'],MotionConfig(**header['motion_config']))
        scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
            epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
        target=.75 if record['id'] in ('r457:1590','r457:1986') else record['expected']['speed_mps']
        context={}
        ps.motion_region(scene,max(0,record['current']['speed_mps']),target,
            record['expected']['steering_rad'],record['current']['previous_steering_rad'],
            record['current']['dt_s'],cfg,ps.lateral_config(cfg),context=context)
        helper_compare(record['id'],(scene,scan,context['queries'][context['masked']],context['padding'],
            candidate.parse_sectors(cfg.known_structure_sectors_json)))
    test_path=ROOT/'test_current_structure_shadow_reference.py'
    tree=ast.parse(test_path.read_bytes())
    tree.body=[n for n in tree.body if not(isinstance(n,ast.Import) and any(a.name=='pytest' for a in n.names))]
    for n in tree.body:
        if isinstance(n,ast.FunctionDef):n.decorator_list=[]
    ns={'__name__':'r470_original_assertions'}
    exec(compile(ast.fix_missing_locations(tree),str(test_path),'exec'),ns)
    count=[0]
    def checked(*args):
        count[0]+=1
        return helper_compare('original_assertion_'+str(count[0]),args)
    ns['current_mask_shadow']=checked
    calls=[('test_narrow_shadow_is_risk_not_new_free_measurement',()),
        *[('test_near_or_missing_flank_is_not_permission',(v,)) for v in (.14,math.inf,math.nan)],
        ('test_wide_distant_shadow_is_rejected',()),('test_undeclared_mask_is_rejected',()),
        ('test_visible_return_inside_nominal_sector_is_not_masked_as_structure',()),
        *[('test_invalid_padding_is_rejected',(v,)) for v in (math.nan,-.01)]]
    for name,args in calls:ns[name](*args)
    assert count[0]==9 and len(helper_checks)==13
    report=dict(passed=True,kernel_checks=kernel_checks,raw_buffer_checks=raw_checks,
        helper_checks=helper_checks,original_assertions=9,pytest_used=False,
        fallback_python_preserved=True,missing_entrypoint_rejected=missing_rejected,
        native_binary_sha256=hashlib.sha256(Path(extension.__file__).read_bytes()).hexdigest(),
        existing_assertion_sha256=hashlib.sha256(test_path.read_bytes()).hexdigest(),
        full_evaluator_calls=0,sensors_opened=False)
    (ROOT/'results/native_verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(passed=True,kernel_cases=len(kernel_checks),raw_rejections=len(raw_checks),helper_cases=len(helper_checks))))


if __name__=='__main__':main()
