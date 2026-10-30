import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from scenario import Scenario

def test_no_trigger_means_stationary_and_time_queries_do_not_trigger():
 s=Scenario('pullout',1.15)
 assert s.pose_at(100)==(1.2,0.,0.) and s.start is None
 s.observe_ego(1.,(0.,0.,0.));assert s.start is None

def test_single_trigger_and_past_queries():
 s=Scenario('pullout',1.15);s.observe_ego(2.,(1.1,0.,0.))
 assert s.start==2. and s.pose_at(1.9)==(1.2,0.,0.)
 s.observe_ego(3.,(2.,0.,0.));assert s.start==2.
 assert s.pose_at(3.5)==s.pose_at(100.)

def test_pullout_pulse_stays_inside_walls_and_stops():
 s=Scenario('pullout',1.15);s.observe_ego(2.,(1.1,0.,0.))
 x,y,yaw=s.pose_at(5.)
 assert 1.5<x<1.6 and -.15<y<-.13 and yaw==-.6
 assert abs(y)+abs(math.sin(yaw))*.1+abs(math.cos(yaw))*.075<.425

def test_slide_is_separate_and_front_block_is_actual_second_box():
 s=Scenario('slide',1.15,True);s.observe_ego(2.,(1.1,0.,0.))
 assert s.pose_at(10)==(1.2,-.2,0.)
 boxes=s.boxes(10);assert len(boxes)==6 and boxes[-2].x_m==1.85
 assert s.description()['physical_path'] is False
