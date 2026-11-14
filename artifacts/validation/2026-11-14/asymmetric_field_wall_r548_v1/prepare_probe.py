"""r525 ray/시험 절차를 두 비대칭 대표로만 준비한다. 실행은 probe.py 한 번."""
from pathlib import Path
import ast
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
BASE=A.parent/'field_large_walls_r525_v1'

def change(text,old,new):
    assert text.count(old)==1,old
    return text.replace(old,new)

def main():
    assert not (A/'probe.py').exists()
    generator=(BASE/'sensor_generator.py').read_text(encoding='utf8')
    generator=change(generator,'    center = shift + amplitude*np.sin(math.tau*xs/1.0)\n    segments = []\n    for sign in (-1.,1.):\n        points = np.c_[xs,center+sign*CFG.wall_offset_m]',
        '    # 좌벽만100mm 안쪽, 좌/우 사인 위상0/pi/2. 모두 미측정 민감도 가정.\n    walls_y = {1.: CFG.wall_offset_m-.10+amplitude*np.sin(math.tau*xs),\n              -1.: -CFG.wall_offset_m+amplitude*np.sin(math.tau*xs+math.pi/2)}\n    segments = []\n    for sign in (-1.,1.):\n        points = np.c_[xs,walls_y[sign]]')
    generator=change(generator,'np.array([xs[index], center[index]-CFG.wall_offset_m])','np.array([xs[index], walls_y[-1.][index]])')
    generator=change(generator,'np.array([xs[index], center[index]+CFG.wall_offset_m])','np.array([xs[index], walls_y[1.][index]])')
    (A/'sensor_generator.py').write_text(generator,encoding='utf8')
    old=(BASE/'probe.py').read_text(encoding='utf8')
    helpers=old[old.index('def summarize(out):'):old.index('cases=[dict(')]
    loop=old[old.index('results=[];terminal_plan.install()'):old.index("assert results[-1]['moving_after'")]
    loop=loop.replace('    with released_plan.enabled():','    with released_plan.enabled():')
    loop=change(loop,"        for case in cases:\n            kw=", "        for case in cases:\n            started=time.perf_counter()\n            kw=")
    loop=change(loop,'            assert len(raw.ranges)==500 and len(scene.rays)==500',
        "            assert len(raw.ranges)==500 and len(scene.rays)==500\n            assert [r.source_index for r in scene.rays]==list(range(500))\n            assert [r.status for r in scene.rays]==[r.status for r in preserved_scene.rays]")
    loop=change(loop,"            results.append(row)","            row.update(left_inward_m=.10,left_phase_rad=0.,right_phase_rad=math.pi/2,elapsed_windows_s=time.perf_counter()-started)\n            results.append(row)")
    head='''"""r548 두 비대칭 벽 대표. 현재 공통 소스, 측정 count 조향 상한. 장치/ROS 없음."""
from pathlib import Path
from dataclasses import replace
import copy,hashlib,importlib.util,json,math,sys,time,os
from collections import Counter
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
if sys.platform=='win32':
    import ctypes
    ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(),0x4000)
import numpy as np
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.dont_write_bytecode=True
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(A.parent/'field_wall_irregularity_r518_v1/test_deps')]
old_cross=np.cross
def compat(a,b,*args,**kwargs):
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.shape[-1]==bb.shape[-1]==2:return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return old_cross(a,b,*args,**kwargs)
np.cross=compat
spec=importlib.util.spec_from_file_location('r521_test_current',ROOT/'tests/test_current_wall_reacquire.py')
t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
t.CFG=replace(t.CFG,max_steering_rad=.28475436568260193)
from arena_autonomy.follow_trace import freeze,comparable_decision,equal_output
from arena_autonomy import current_wall_reacquire as reacquire
from arena_autonomy.race_runtime_policy import terminal_plan,released_plan
generator=(A/'sensor_generator.py').read_text(encoding='utf8')
namespace=dict(t.__dict__);exec(compile(generator,str(A/'sensor_generator.py'),'exec'),namespace);sensor=namespace['sensor']
'''
    cases="cases=[dict(name='asym60_gap50_left_in100',amplitude=.06,gap=.05,shift=0.),\n       dict(name='asym60_gap50_left_in100_actual_obstacle',amplitude=.06,gap=.05,shift=0.,obstacle=True)]\n"
    end='''
source_paths={ROOT/'tests/test_current_wall_reacquire.py',A.parent/'field_large_walls_r525_v1/sensor_generator.py',A.parent/'field_large_walls_r525_v1/probe.py',ROOT/'artifacts/validation/2026-11-14/measured_steering_apply_r541_v1/selected_v2/derived_limits.json'}
for module in list(sys.modules.values()):
    name=getattr(module,'__file__',None)
    if name:
        path=Path(name).resolve()
        if path.is_file() and path.suffix=='.py' and path.is_relative_to(ROOT/'src'):
            source_paths.add(path)
manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(source_paths)}
(A/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\\n',encoding='utf8')
report=dict(scope='two assumed asymmetric wall shapes; original r525 local and active-pass fixture protocols, not closed-loop',
    numpy_version=np.__version__,compatibility='process-local 2D determinant; Python numeric default; no installed dependency changes',
    command_limit_rad=t.CFG.max_steering_rad,full_follow_config=vars(t.CFG),cases=results,WSL=False,Jetson=False,Gazebo=False,
    obstacle_preserved_zero=all(results[-1][key]['speed_mps']==0 for key in ('local_after','moving_after','helper')),
    common_file_changes=False,success_search_repeated=False,
    source_hashes=manifest)
(A/'report.json').write_text(json.dumps(freeze(report),indent=2)+'\\n',encoding='utf8')
'''
    source=head+helpers+cases+loop+end
    ast.parse(source);ast.parse(generator)
    (A/'probe.py').write_text(source,encoding='utf8')
    print('prepared two cases; probe not yet executed')
if __name__=='__main__':main()
