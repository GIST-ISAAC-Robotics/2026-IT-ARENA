"""원 helper와 후보의 전체 exact 및 기존 9개 assert를 Windows에서 검사한다."""
import ast
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import tracemalloc

A=Path(__file__).resolve().parent
O=A/'optimization_v1'
sys.path[:0]=[str(O/'stage/src/arena_autonomy'),str(O/'stage/src/arena_vehicle_interface')]
import numpy as np
from arena_autonomy import known_structure as candidate,pass_release_trial as ps
from arena_autonomy.follow_trace import restore_call,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.rear_fringe_trial import _query_brackets

original_file=A/'bundle/stage/src/arena_autonomy/arena_autonomy/known_structure.py'
spec=importlib.util.spec_from_file_location('r467_original_structure',original_file)
original=importlib.util.module_from_spec(spec)
spec.loader.exec_module(original)
comparisons=[]
def compare(label,args):
    before=original.current_mask_shadow(*args)
    after=candidate.current_mask_shadow(*args)
    row=dict(label=label,exact=before==after,original=before,optimized=after)
    comparisons.append(row)
    assert row['exact'],row
    return after

def main():
    data=json.loads((A/'bundle/inputs.json').read_bytes())
    bracket_checks=[];largest=None
    for record in data['records']:
        header=data['headers'][record['dataset']]
        cfg=FollowConfig(**header['follow_config'])
        info,scan,history=restore_call(record['current'],MotionConfig(**header['motion_config']))
        scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
            epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
        target=.75 if record['id'] in ('r457:1590','r457:1986') else record['expected']['speed_mps']
        context={}
        region=ps.motion_region(scene,max(0,record['current']['speed_mps']),target,
            record['expected']['steering_rad'],record['current']['previous_steering_rad'],
            record['current']['dt_s'],cfg,ps.lateral_config(cfg),context=context)
        q=context['queries'][context['masked']]
        args=(scene,scan,q,context['padding'],candidate.parse_sectors(cfg.known_structure_sectors_json))
        compare(record['id'],args)
        origins=np.asarray([r.ray_origin_xy_m for r in scene.rays])
        directions=np.asarray([r.direction_xy for r in scene.rays])
        expected=list(_query_brackets(q,origins,directions))
        actual=[(radius[i],angles[i],int(lo[i]),int(hi[i]))
            for radius,angles,lo,hi in candidate._current_mask_query_batches(q,origins,directions)
            for i in range(len(radius))]
        exact=all(np.array_equal(x[0].view(np.uint64),y[0].view(np.uint64))
            and np.array_equal(x[1].view(np.uint64),y[1].view(np.uint64)) and x[2:]==y[2:]
            for x,y in zip(expected,actual)) and len(expected)==len(actual)
        assert exact,record['id']
        bracket_checks.append(dict(id=record['id'],queries=len(q),rays=len(scene.rays),
            returned_rays=sum(r.status=='return' for r in scene.rays),all_bracket_bits_exact=exact,
            local_numpy_region=region,full_evaluator_run=False))
        if largest is None or len(q)>len(largest[2]):largest=args
    # pytest가 없는 기존 Windows 환경이다. 원 test body는 그대로 실행하고,
    # pytest import/parametrize decorator만 AST에서 제외해 9개 명시 인수를 호출한다.
    test_path=A.parents[0]/'../2026-11-13/solo_current_mask_r463_v1/test_current_structure_shadow.py'
    test_path=test_path.resolve()
    tree=ast.parse(test_path.read_bytes())
    tree.body=[node for node in tree.body if not(isinstance(node,ast.Import)
        and any(alias.name=='pytest' for alias in node.names))]
    for node in tree.body:
        if isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef)):node.decorator_list=[]
    namespace={'__name__':'r467_pure_test_assertions'}
    exec(compile(ast.fix_missing_locations(tree),str(test_path),'exec'),namespace)
    counter=[0]
    def checked(*args):
        counter[0]+=1
        return compare('original_test_assertion_'+str(counter[0]),args)
    namespace['current_mask_shadow']=checked
    calls=[('test_narrow_shadow_is_risk_not_new_free_measurement',()),
        *[('test_near_or_missing_flank_is_not_permission',(v,)) for v in (.14,math.inf,math.nan)],
        ('test_wide_distant_shadow_is_rejected',()),('test_undeclared_mask_is_rejected',()),
        ('test_visible_return_inside_nominal_sector_is_not_masked_as_structure',()),
        *[('test_invalid_padding_is_rejected',(v,)) for v in (math.nan,-.01)]]
    for name,args in calls:namespace[name](*args)
    assert counter[0]==9
    scene,scan,q,pad=namespace['example']()
    sectors=namespace['SECTORS']
    far=np.array([[.0754,0.]])+ .4*np.array([[math.cos(math.radians(-123.15)),math.sin(math.radians(-123.15))]])
    for label,queries in (
        ('batch_boundary_first_wide',np.vstack((np.tile(q,(16,1)),far,q))),
        ('first_bad_priority',np.vstack((far,[[.0754,0.]],q))),
        ('first_bad_priority_reversed',np.vstack(([[.0754,0.]],far,q))),
        ('first_max_tie_order',np.tile(q[::-1],(35,1))),
        ('empty_queries',np.empty((0,2)))):
        compare(label,(scene,scan,queries,pad,sectors))
    # 부호 있는 0을 포함해 dot/angle의 원 연산 비트를 확인한다.
    q=np.array([[0.,0.],[-0.,-0.],[1.,0.],[-1.,-0.]])
    origins=np.zeros((4,2));directions=np.array([[0.,0.],[-0.,-0.],[1.,0.],[-1.,-0.]])
    expected=list(_query_brackets(q,origins,directions))
    actual=next(candidate._current_mask_query_batches(q,origins,directions))
    assert all(np.array_equal(x[1].view(np.uint64),actual[1][i].view(np.uint64))
        and x[2:]==(int(actual[2][i]),int(actual[3][i])) for i,x in enumerate(expected))
    allocation={}
    for label,function in (('original',original.current_mask_shadow),('optimized',candidate.current_mask_shadow)):
        tracemalloc.start()
        function(*largest)
        current,peak=tracemalloc.get_traced_memory()
        tracemalloc.stop()
        allocation[label]=dict(peak_traced_bytes=peak,current_traced_bytes=current,
            timing_measured=False,scope='Windows Python/NumPy allocations during one helper; not ARM RSS')
    report=dict(passed=True,helper_comparisons=len(comparisons),comparisons=comparisons,
        original_tests_passed=len(calls),original_tests_sha256=hashlib.sha256(test_path.read_bytes()).hexdigest(),
        original_test_bodies_unchanged=True,pytest_used=False,bracket_checks=bracket_checks,
        signed_zero_bracket_bits_exact=True,local_allocations=allocation,
        physical_measurement=False,whole_evaluator=False,
        note='Windows NumPy geometry may differ in count from native ARM; original/optimized use identical local arrays')
    (O/'local_verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('comparisons','bracket_checks')}))


if __name__=='__main__':main()
