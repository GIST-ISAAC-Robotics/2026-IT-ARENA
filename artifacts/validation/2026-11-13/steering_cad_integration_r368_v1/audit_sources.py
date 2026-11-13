"""후속 수정이 적용된 실제 제품 파일과 원 CAD의 출처를 기록한다. 장치 I/O 없음."""
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
FILES=[
 'src/arena_vehicle_interface/arena_vehicle_interface/steering_calibration.py',
 'src/arena_vehicle_interface/arena_vehicle_interface/steering_cad_table.py',
 'src/arena_vehicle_interface/portable/cad_steering.hpp',
 'src/arena_vehicle_interface/portable/cad_steering_table.hpp',
 'src/arena_vehicle_interface/portable/cad_servo_goal.hpp',
 'src/arena_vehicle_interface/portable/steering_zero.hpp',
 'src/arena_gazebo/src/single_motor_drive.cpp','src/arena_gazebo/test/test_cad_steering.cpp',
 'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml',
 'src/arena_description/models/arena_car/model.sdf.xacro',
 'src/arena_bringup/launch/simulation.launch.py','scripts/steering_zero_profile.py',
 'tests/test_steering_calibration.py',
 'artifacts/validation/2026-11-13/steering_cad_fusion_v1/steering_table.csv',
]
def main():
    report=dict(scope='current product sources, not the earlier staged patch or measured hardware',
                sources={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in FILES})
    (A/'product_sources.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('recorded',len(FILES),'sources')
if __name__=='__main__':main()
