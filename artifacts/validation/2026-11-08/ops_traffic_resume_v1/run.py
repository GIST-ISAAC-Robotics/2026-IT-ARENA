"""평가 전용: 정지 앞차가 있는 상태의 HTTP STOP -> RESET 없는 START."""
from pathlib import Path
import hashlib,json,math,subprocess,sys,time
import race_ops_eval as base
import validate_basic_autonomy as launch
from validate_g1_traffic import actor_sdf
from settle import hold_window
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]
registered=json.loads((HERE/'registered.json').read_text())
assert not (HERE/'run1').exists()
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in registered['sources'].items())
class TrafficStopStart(base.Scenario):
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
        try:self.run_steps()
        except (base.Stop,KeyboardInterrupt,OSError,ValueError,KeyError,TypeError,subprocess.SubprocessError):
            try:self.stop_and_measure('failure_stop_hold')
            except Exception as error:self.note('failure_stop_error',error=str(error))
            raise
    def run_steps(self):
        for name,topic in [('actor_poses','/model/eval_g1b_actor/pose'),('contacts','/eval/g1b_actor/contact')]:
            with (self.output/(name+'.jsonl')).open('w',encoding='utf-8') as stream:
                self.processes.append(subprocess.Popen(['gz','topic','-e','-t',topic,'--json-output'],cwd=ROOT,
                    stdin=subprocess.DEVNULL,stdout=stream,stderr=self.log,start_new_session=True))
        self.first_start()
        probe=subprocess.run(['gz','topic','-i','-t','/eval/g1b_actor/contact'],capture_output=True,text=True,timeout=10)
        self.note('contact_source',returncode=probe.returncode,output=probe.stdout,stderr=probe.stderr,reader_alive_at_graph_audit=self.processes[-1].poll() is None)
        binding=dict(self.ops.state['binding'])
        self.stop_and_measure('first_stop_hold')
        self.wait(lambda s:s['controls']['resume_start']['allowed'],3.,'START allowed while stopped')
        at=self.truth.latest()
        self.note('before_resume',stationary_ready=self.ops.state['mcu']['stationary_ready'])
        entry=self.press('resume_start')
        state=self.resume('traffic_without_reset',entry)
        current=state['binding']
        self.note('same_race_resumed',initial_binding=binding,current_binding=current,start_source=state['race']['start_source'])
        if any(current.get(k)!=binding.get(k) for k in ('session','race_id','race_epoch')):
            raise base.Stop('race/session changed')
        deadline=time.monotonic()+25*self.scale
        while time.monotonic()<deadline:
            t,x,y,yaw,_=self.truth.latest()
            progress=(x-at[1])*math.cos(at[3])+(y-at[2])*math.sin(at[3])
            if progress>=1.2:
                self.note('post_resume_progress',progress_m=progress,from_sim_s=at[0],elapsed_s=t-at[0])
                break
            time.sleep(.025)
        else:raise base.Stop('traffic resume did not progress 1.2m within test time')
        self.stop_and_measure('final_stop_hold')
        self.note('traffic_ops_complete',scope='one stationary front actor; no RESET, no ego reposition')
base.Scenario=TrafficStopStart
base.REGISTERED_SCENARIOS={**base.REGISTERED_SCENARIOS,'official_v2026_11_06':('representative',)}
base.PROFILE='g1_open_1p5'
original=launch.g1_solo_arguments

def selected(*args,**kwargs):
    output=args[4]
    actor=output/'parked_actor.sdf'
    cfg=json.loads((ROOT/'config/tests/official_traffic_v1.json').read_text())
    pose=base.Course(ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06').pose(43.26)
    actor.write_text(actor_sdf(cfg,pose),encoding='utf-8')
    (output/'actor_registered.json').write_text(json.dumps(dict(s_m=43.26,pose=pose,actor=cfg['actor']),indent=2))
    flags=['g1_piecewise_wall_trial','evaluation_handoff_yield_progress','evaluation_handoff_rear_sweep',
           'evaluation_handoff_observed_motion','evaluation_side_departure_trial','evaluation_current_lane_preference_trial',
           'evaluation_handoff_continuous_pace','evaluation_moving_pass_trial','evaluation_handoff_wall_witness']
    return original(*args,**kwargs)+[k+':=true' for k in flags]+[
        'pilot_marker_z:=0.140','evaluation_handoff_reserve:=paced','evaluation_grass_alignment:=road_level',
        'evaluation_actor_sdf:='+str(actor)]
launch.g1_solo_arguments=selected
sys.argv=[__file__,'--output',str(HERE/'run1'),'--track','official_v2026_11_06','--local-sensor-config',
    str(ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'),'--scenario','representative',
    '--first-stop-s','42.26','--wall-limit-s','1200','--wall-scale','25']
raise SystemExit(base.main())
