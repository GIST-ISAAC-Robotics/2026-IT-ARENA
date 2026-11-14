"""선택 count/CAD 제한의 차량 YAML 연결. Windows 순수 검사, 장치 I/O 없음."""
from pathlib import Path
import difflib, hashlib, json, math, re, struct, sys, unittest
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'src/arena_autonomy')]
from arena_vehicle_interface.steering_calibration import CadMapping, read_profile, cad_servo_goal
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig
from arena_vehicle_interface.applied_guard import AppliedGuard
from arena_autonomy.vehicle_geometry import equivalent_steering_limit, float32_limit_not_above
PROFILE = ROOT/'artifacts/validation/2026-11-14/measured_steering_apply_r541_v1/selected_v2/steering_zero.selected.json'
BASE = ROOT/'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'
DEST = HERE/'vehicle.measured_steering_2026_11_14.yaml'
m = CadMapping.default()
p = read_profile(PROFILE)
ends = [m.relative((p[k]-p['zero_count'])*2*math.pi/(p['count_direction_per_cad_positive']*p['counts_per_turn'])) for k in ('safe_min_count','safe_max_count')]
C = min(-ends[0].equivalent_rad, ends[1].equivalent_rad)
L = float32_limit_not_above(C)
geom = dict(wheelbase=.128, track=.1239, wheel_limit=math.pi/6)
# Invert the count-derived physical bound before float32 rounding. Inverting L
# instead loses one extra float32 ULP through the CAD binary-search round trip.
joint = max(abs(v) for s in (-1,1) for g in [m.central(s*C,**geom)] for v in (g.left_rad,g.right_rad))
drive = dict(simulation_drive_mode='single_motor',simulation_steering_model='fusion_cad_2026_11_13',wheelbase_m=.128,track_width_m=.1239,max_steering_angle_rad=joint)
physical = equivalent_steering_limit(drive)
pattern = rb'(?m)^(    max_steering_angle_rad: )[^\r\n]+'
original = BASE.read_bytes()
replacement = ('    max_steering_angle_rad: '+repr(joint)+' # count2600..2980-derived CAD joint cap; wheel angles unmeasured').encode()
derived, edits = re.subn(pattern,replacement,original)
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def next32(v): return struct.unpack('<f',struct.pack('<I',struct.unpack('<I',struct.pack('<f',v))[0]+1))[0]
def receiver(limit):
    r=ActuationReceiver(ContractConfig(max_steering_rad=limit),boot_id='r542');h=CommandProducer(client_id='r542')
    for step in range(23):
        t=step*10000;r.update_encoder(t,sequence=step,capture_us=t,left_ticks=0,right_ticks=0);r.tick(t)
    assert r.receive(h.control(r.offer(t),'ARM'),t)[0]
    return r,h,t
def status(t,angle=0.,state='ARMED',speed=0.):
    return dict(now_us=t,boot='r542',generation=1,state=state,target_speed_mps=speed,steering_rad=angle,deadline_us=t+100000,reason='test')
class Checks(unittest.TestCase):
    def test_profile_boundary(self):
        self.assertEqual(tuple(p[k] for k in ('zero_count','count_direction_per_cad_positive','counts_per_turn','safe_min_count','safe_max_count')),(2811,-1,4096,2600,2980));self.assertFalse(p['hardware_angle_measured'])
    def test_only_one_yaml_value_changes(self):
        self.assertEqual(edits,1);self.assertEqual(re.sub(pattern,b'LIMIT',original),re.sub(pattern,b'LIMIT',derived));self.assertEqual(DEST.read_bytes(),derived)
    def test_exact_sender_roundtrip(self):
        self.assertEqual(L,.28475436568260193);self.assertEqual(float32_limit_not_above(physical),L);self.assertLessEqual(L,physical);self.assertLessEqual(physical,C);self.assertGreater(next32(L),physical)
    def test_wrong_central_as_joint_is_detected(self):
        self.assertLess(equivalent_steering_limit({**drive,'max_steering_angle_rad':L}),L-.01)
    def test_count_and_wheel_sign(self):
        goals=[cad_servo_goal(p,s*L,command_limit=L,**geom) for s in (-1,0,1)]
        self.assertEqual([g['position'] for g in goals],[2643,2811,2980]);self.assertLess(goals[0]['left_target_rad'],0);self.assertGreater(goals[2]['left_target_rad'],0)
    def test_joint_goals_inside(self):
        for s in (-1,1):
            g=m.central(s*L,**{**geom,'wheel_limit':joint});self.assertFalse(g.clamped);self.assertLessEqual(max(abs(g.left_rad),abs(g.right_rad)),joint);self.assertAlmostEqual(g.equivalent_rad,s*L,places=11)
    def test_real_and_virtual_strict_receiver(self):
        for limit in (L,physical):
            for s in (-1,1):
                for angle,accepted in ((s*L,True),(s*next32(L),False)):
                    r,h,t=receiver(limit);f=h.drive(r.offer(t),speed_mps=.2,steering_rad=angle,source_us=t,host_now_us=t);self.assertEqual(r.receive(f,t)[0],accepted)
    def test_sim_guard_bounds(self):
        for s in (-1,1):
            g=AppliedGuard(max_angle=physical);self.assertTrue(g.update(status(1000),1000,1.));self.assertTrue(g.update(status(2000,s*L,'ACTIVE',.2),2000,1.001));self.assertFalse(g.update(status(3000,s*next32(L),'ACTIVE',.2),3000,1.002));self.assertEqual(g.speed,0.)
    def test_stop_and_expiry_preserved(self):
        g=AppliedGuard(max_angle=physical);self.assertTrue(g.update(status(1000),1000,1.));self.assertTrue(g.update(status(2000,L,'ACTIVE',.2),2000,1.001));g.watch(102000,1.101);self.assertEqual(g.speed,0.);self.assertTrue(g.blocked)
        r,h,t=receiver(L);self.assertTrue(r.receive({'v':1,'kind':'STOP'},t)[0]);self.assertEqual(r.target_speed,0.)
    def test_panel_remains_lower(self): self.assertEqual(min(.2,L),.2)
def main():
    assert edits==1
    DEST.write_bytes(derived)
    (HERE/'vehicle_config.patch').write_text(''.join(difflib.unified_diff(original.decode().splitlines(True),derived.decode().splitlines(True),fromfile=BASE.relative_to(ROOT).as_posix(),tofile=DEST.relative_to(ROOT).as_posix())),encoding='utf-8',newline='')
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Checks))
    sources=[BASE,PROFILE,ROOT/'src/arena_vehicle_interface/arena_vehicle_interface/steering_calibration.py',ROOT/'src/arena_vehicle_interface/arena_vehicle_interface/steering_cad_table.py',ROOT/'src/arena_autonomy/arena_autonomy/vehicle_geometry.py',ROOT/'src/arena_bringup/launch/simulation.launch.py',ROOT/'src/arena_gazebo/src/single_motor_drive.cpp']
    report=dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),python=sys.version,platform=sys.platform,source_hashes={x.relative_to(ROOT).as_posix():digest(x) for x in sources},vehicle_config=DEST.relative_to(ROOT).as_posix(),vehicle_config_sha256=digest(DEST),command_limit_rad=L,raw_count_central_limit_rad=C,joint_limit_rad=joint,launch_physical_limit_rad=physical,launch_sender_limit_rad=float32_limit_not_above(physical),manual_steering_limit_rad=min(.2,L),right_unused_count=43,hardware_angle_measured=False,device_access=False,ros_gazebo_run=False,limitation='Source/numeric/transport checks only; no ROS CDR, C++ execution, Gazebo or installed output.')
    (HERE/'audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('tests','failures','errors','vehicle_config_sha256','command_limit_rad','joint_limit_rad')},indent=2))
    return 0 if result.wasSuccessful() else 1
if __name__=='__main__': raise SystemExit(main())
