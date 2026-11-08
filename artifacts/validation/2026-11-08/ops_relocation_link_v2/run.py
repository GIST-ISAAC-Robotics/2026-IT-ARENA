"""새 코스 정지 정착·평가 전용 재배치 후 START 대표. 원 운영 검사기의 HTTP/정지/종료 도구를 재사용한다."""
from pathlib import Path
import hashlib,json,sys,time
import race_ops_eval as base
import validate_basic_autonomy as launch
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
registered=json.loads((HERE/'registered.json').read_text())
assert not (HERE/'run1').exists()
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in registered['sources'].items())
from settle import hold_window
import math
class NewCourseStopStart(base.Scenario):
    def stop_and_measure(self,name):
        self.stop_car()
        # stop_car records software confirmation and coarse truth.speed() (<.02);
        # also keep its complete first half-second for comparison with the failed v1 gate.
        note=self.steps[-1]
        first=float(note['truth'][0])
        press=next(s for s in reversed(self.steps) if s['step']=='press_stop')
        press_t=float(press['truth'][0])
        deadline=time.monotonic()+6*self.scale
        initial_done=False
        while time.monotonic()<deadline:
            tail=self.truth.rows[-2500:]
            w=[r for r in tail if r[0]>=first]
            if w and w[-1][0]-first>=.5 and not initial_done:
                stop_index=next(i for i,r in enumerate(w) if r[0]-first>=.5)
                initial=hold_window(w[:stop_index+1])
                self.note(name+'_original_window',after_coarse_s=first-press_t,window=initial,
                          coarse_stationary_ready=note.get('stationary_ready'))
                initial_done=True
            settled=hold_window(w)
            if settled and settled['passed'] and initial_done:
                if settled['to_sim_s']-press_t>1.6:raise base.Stop('dense STOP hold exceeded 1.6s from request')
                self.note(name,**settled,from_stop_request_s=settled['to_sim_s']-press_t,
                          stationary_ready=self.ops.state['mcu']['stationary_ready'])
                return
            if w and w[-1][0]-press_t>1.6:break
            time.sleep(.02)
        raise base.Stop('0.5s dense hold not measured within 1.6s from STOP')
    def run(self):
        self.first_start()
        binding=dict(self.ops.state['binding'])
        self.stop_and_measure('first_stop_hold')
        # RESET is a separate stationary operation. No map/placement is sent to the vehicle.
        self.stage(self.press('recovery_reset'), ('policy_accepted',))
        self.wait(lambda s:s['controls']['resume_start']['allowed'],3.,'START ready in stopped session')
        self.note('before_place',stationary_ready=self.ops.state['mcu']['stationary_ready'])
        pose=base.teleport(self.course,self.truth,12.,self.log)
        self.note('requested_place',**pose)
        deadline=time.monotonic()+2*self.scale
        while time.monotonic()<deadline:
            t,x,y,yaw,_=self.truth.latest()
            if (math.hypot(x-pose['x'],y-pose['y'])<.04 and
                abs(math.atan2(math.sin(yaw-pose['yaw']),math.cos(yaw-pose['yaw'])))<.05):break
            time.sleep(.01)
        else:raise base.Stop('new placement not observed')
        self.note('placement_verified',target=pose)
        entry=self.press('resume_start')
        state=self.resume('relocated_straight',entry)
        current=state['binding']
        self.note('same_race_resumed',initial_binding=binding,current_binding=current,start_source=state['race']['start_source'])
        if any(current.get(k)!=binding.get(k) for k in ('session','race_id','race_epoch')):
            raise base.Stop('race/session changed')
        at=self.truth.latest()
        deadline=time.monotonic()+20*self.scale
        while True:
            t,x,y,yaw,_=self.truth.latest()
            dx,dy=x-pose['x'],y-pose['y']
            longitudinal=dx*math.cos(pose['yaw'])+dy*math.sin(pose['yaw'])
            lateral=-dx*math.sin(pose['yaw'])+dy*math.cos(pose['yaw'])
            if longitudinal>=1.2:break
            if time.monotonic()>deadline:raise base.Stop('did not make progress along new heading')
            time.sleep(.03)
        self.note('post_resume_progress',progress_m=longitudinal,lateral_m=lateral,
                  from_sim_s=at[0],target=pose)
        self.stop_and_measure('final_stop_hold')
        self.note('newcourse_simple_ops_complete',scope='simulated judge reposition, new heading, same race')
base.Scenario=NewCourseStopStart
base.REGISTERED_SCENARIOS={**base.REGISTERED_SCENARIOS,'official_v2026_11_06':('representative',)}
base.PROFILE='g1_open_1p5'
original=launch.g1_solo_arguments
def selected(*args,**kwargs):return original(*args,**kwargs)+['g1_piecewise_wall_trial:=true','pilot_marker_z:=0.140']
launch.g1_solo_arguments=selected
sys.argv=[__file__,'--output',str(HERE/'run1'),'--track','official_v2026_11_06','--local-sensor-config',str(ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'),'--scenario','representative','--first-stop-s','42.66','--wall-limit-s','1200','--wall-scale','25']
raise SystemExit(base.main())
