"""새 코스 최소 운영 대표. 원 운영 검사기의 HTTP/정지/종료 도구를 재사용한다."""
from pathlib import Path
import hashlib,json,sys,time
import race_ops_eval as base
import validate_basic_autonomy as launch
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
registered=json.loads((HERE/'registered.json').read_text())
assert not (HERE/'run1').exists()
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in registered['sources'].items())
class NewCourseStopStart(base.Scenario):
    def stable_hold(self,name):
        start=self.truth.latest();self.hold(.5);end=self.truth.latest()
        self.note(name,from_sim_s=start[0],to_sim_s=end[0],drift_m=base.math.dist(start[1:3],end[1:3]))
        if end[0]-start[0]<.49 or base.math.dist(start[1:3],end[1:3])>.02:raise base.Stop('stationary hold not established')
    def run(self):
        self.first_start()
        binding=dict(self.ops.state['binding']);self.stop_car();self.stable_hold('first_stop_hold')
        self.wait(lambda s:s['controls']['resume_start']['allowed'],3.,'START without RESET allowed')
        at=self.truth.latest();entry=self.press('resume_start');state=self.resume('same_position_no_reset',entry)
        self.note('same_race_resumed',initial_binding=binding,current_binding=state['binding'],start_source=state['race']['start_source'])
        if any(state['binding'][k]!=binding[k] for k in ('session','race_epoch')):raise base.Stop('race/session changed')
        deadline=time.monotonic()+20*self.scale
        while base.math.dist(at[1:3],self.truth.latest()[1:3])<1.2:
            if time.monotonic()>deadline:raise base.Stop('resume progress timeout')
            time.sleep(.05)
        self.note('post_resume_progress',progress_m=base.math.dist(at[1:3],self.truth.latest()[1:3]),from_sim_s=at[0])
        self.stop_car();self.stable_hold('final_stop_hold')
        self.note('newcourse_simple_ops_complete',scope='RGB start, same-position STOP/START without RESET, G1 only')
base.Scenario=NewCourseStopStart
base.REGISTERED_SCENARIOS={**base.REGISTERED_SCENARIOS,'official_v2026_11_06':('representative',)}
base.PROFILE='g1_open_1p5'
original=launch.g1_solo_arguments
def selected(*args,**kwargs):return original(*args,**kwargs)+['g1_piecewise_wall_trial:=true','pilot_marker_z:=0.140']
launch.g1_solo_arguments=selected
sys.argv=[__file__,'--output',str(HERE/'run1'),'--track','official_v2026_11_06','--local-sensor-config',str(ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'),'--scenario','representative','--first-stop-s','42.66','--wall-limit-s','1200','--wall-scale','25']
raise SystemExit(base.main())
