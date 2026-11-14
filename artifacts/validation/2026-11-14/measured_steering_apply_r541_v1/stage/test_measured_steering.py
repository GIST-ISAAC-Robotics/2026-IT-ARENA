"""실측 프로필의 순수 변환/기존 설치 경로 검사. 임시 board는 합성 fixture이며 채택하지 않는다."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import prepare_measured_steering as subject
sys.path.insert(0,str(subject.ROOT/'tests'))
from test_runtime_installed_limits import synthetic_bundle,BASE
import runtime_installed_limits as limits
from arena_vehicle_interface.steering_calibration import relative_goal
from arena_autonomy.vehicle_geometry import equivalent_steering_limit,float32_limit_not_above


class MeasuredSteeringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile=subject.read_profile(Path(__file__).resolve().parents[1]/'selected_v2/steering_zero.selected.json')
        cls.report,cls.header=subject.derive(cls.profile)

    def test_ready_profile_preserves_original_and_records_flag_meanings(self):
        original=subject.read_profile(subject.MEASURED/'straight.json')
        with self.assertRaises(ValueError):subject.check_profile(original,ready=True)
        subject.check_profile(self.profile,ready=True,assembly_id='arena-car-v1')
        self.assertEqual(self.profile['zero_count'],original['zero_count'])
        self.assertEqual((self.profile['safe_min_count'],self.profile['safe_max_count']), (2600,2980))
        self.assertEqual(self.profile['count_direction_per_cad_positive'],-1)
        self.assertFalse(self.profile['hardware_angle_measured'])
        self.assertFalse(self.profile['selection_basis']['manual_operating_limits_confirmed_at_capture'])
        self.assertTrue(self.profile['selection_basis']['operating_limits_selected_now'])
        self.assertEqual(self.profile,subject.selected_profile())
        for name,expected in self.profile['selection_basis']['source_sha256'].items():
            self.assertEqual(subject.sha(subject.ROOT/name),expected)

    def test_401_central_commands_remain_in_selected_counts_and_left_positive(self):
        limit=self.report['command_limit_rad'];positions=[]
        for i in range(401):
            command=limit*(i-200)/200
            goal=subject.cad_servo_goal(self.profile,command,command_limit=limit,**subject.GEOMETRY)
            positions.append(goal['position'])
            self.assertTrue(2600<=goal['position']<=2980)
            self.assertFalse(goal['hardware_angle_measured'])
            if i<200:self.assertLessEqual(goal['central_realized_rad'],0.)
            if i>200:self.assertGreaterEqual(goal['central_realized_rad'],0.)
        self.assertEqual(positions,sorted(positions))
        self.assertEqual((positions[0],positions[200],positions[-1]),(2643,2811,2980))

    def test_count_saturation_and_public_command_rejection_keep_existing_contract(self):
        self.assertEqual(relative_goal(self.profile,-10)['position'],2980)
        self.assertEqual(relative_goal(self.profile,10)['position'],2600)
        self.assertTrue(relative_goal(self.profile,10)['clamped'])
        for value in (self.report['command_limit_rad']+.0001,-self.report['command_limit_rad']-.0001):
            with self.assertRaises(ValueError):subject.cad_servo_goal(self.profile,value,
                command_limit=self.report['command_limit_rad'],**subject.GEOMETRY)
        with self.assertRaises(ValueError):subject.selected_profile(2550,3031)

    def test_float32_and_vehicle_joint_cap_restore_same_central_limit(self):
        limit=self.report['command_limit_rad']
        self.assertEqual(limit,0.28475436568260193)
        self.assertEqual(float32_limit_not_above(limit),limit)
        drive=dict(simulation_steering_model='fusion_cad_2026_11_13',simulation_drive_mode='single_motor',
            wheelbase_m=.128,track_width_m=.1239,max_steering_angle_rad=self.report['sim_joint_inner_limit_rad'])
        self.assertEqual(float32_limit_not_above(equivalent_steering_limit(drive)),limit)
        self.assertEqual(self.report['right_unused_count'],43)
        self.assertAlmostEqual(self.report['right_unused_central_deg'],4.242914501545417)

    def test_existing_board_generator_ros_and_host_share_selected_limit_without_motor_changes(self):
        with tempfile.TemporaryDirectory(prefix='r541-synthetic-board-') as temp:
            root=Path(temp);synthetic=synthetic_bundle(root)
            board=subject.installed.read_json(synthetic/'runtime_board.json')
            board['assembly_id']=self.profile['assembly_id']
            original_motor={k:copy.deepcopy(v) for k,v in board.items() if k!='servo'}
            board['servo']['command_limit']=self.report['command_limit_rad']
            bp=root/'synthetic_board.json';subject.write(bp,board)
            zp=root/'selected_zero.json';subject.write(zp,self.profile)
            bp_raw=bp.read_bytes();zp_raw=zp.read_bytes()
            installed=root/'selected_installed';subject.installed.prepare(bp,zp,board['assembly_id'],installed)
            subject.installed.verify(installed)
            self.assertEqual({k:v for k,v in subject.installed.read_json(installed/'runtime_board.json').items() if k!='servo'},original_motor)
            base=root/'base.yaml';base.write_text(limits.yaml.safe_dump(BASE),encoding='utf-8')
            selected=root/'selected';limits.prepare(installed,base,selected)
            limits.verify(selected,installed,host_steering_rad=self.report['command_limit_rad'])
            params=limits.profile_values((selected/'runtime_profile.yaml').read_bytes())
            for node in limits.NODES:self.assertEqual(params[node]['ros__parameters'][limits.PARAMETER],self.report['command_limit_rad'])
            host=subject.installed.read_json(selected/'runtime_host_limits.json')
            self.assertEqual(host['arguments'],['--max-steering-rad','0.28475436568260193'])
            self.assertEqual((installed/'mounted_steering_zero.hpp').read_text(encoding='ascii'),self.header)
            self.assertEqual((bp.read_bytes(),zp.read_bytes()),(bp_raw,zp_raw))

    def test_current_cli_goal_uses_real_zero_and_explicit_narrow_limit(self):
        profile=Path(__file__).resolve().parents[1]/'selected_v2/steering_zero.selected.json'
        for degrees,side in ((-15,-1),(0,0),(15,1)):
            run=subprocess.run([sys.executable,'-B',str(subject.ROOT/'scripts/steering_zero_profile.py'),
                'goal',str(profile),'--steering-deg',str(degrees),'--command-limit-rad',str(self.report['command_limit_rad'])],
                capture_output=True,text=True,check=True)
            result=json.loads(run.stdout)
            self.assertFalse(result['output_enabled'])
            if side<0:self.assertLess(result['position'],2811)
            elif side>0:self.assertGreater(result['position'],2811)
            else:self.assertEqual(result['position'],2811)


if __name__=='__main__':unittest.main()
