"""현재 ROS IMU 콜백의 회전/잡음 응답. ROS 네트워크·실물/차체 주행은 제외."""
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from sensor_msgs.msg import Imu
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.lidar_motion_ros import MotionInput
from perturb import normal

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'tests'))
from test_lidar_motion_ros import NodeDouble


def probe(case):
    config={'motion_feedback_mode':'drive_motor_shaft','motion_gear_ratio':15.,
            'motion_wheel_radius_m':.0325,'motion_rear_axle_x_m':-.0675,
            'motion_lidar_x_m':.06,'motion_imu_frame_id':'imu_link'}
    node=NodeDouble(config); adapter=MotionInput(node)
    reference=MotionHistory(adapter.history.config); unchecked=MotionHistory(adapter.history.config)
    raw=[]; truth=[]
    for i in range(209):
        t=round(i/208*1e9)/1e9; node.t=t
        true_rate=1.+math.sin(2*math.pi*t)
        measured=true_rate
        if case=='residual': measured+=math.radians(1)+.02*normal(28,'gyro',t.hex())
        elif case=='two_spikes' and i in (104,105): measured+=4.
        elif case=='vibration': measured+=.3*math.sin(2*math.pi*30*t)
        message=Imu(); message.header.frame_id='imu_link'
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(round(t*1e9),10**9)
        message.angular_velocity.z=measured
        adapter.on_imu(message)
        reference.add_gyro(t,true_rate); unchecked.add_gyro(t,measured)
        # gyro만 비교하므로 구동 속도 이력은 같은 0.35 m/s로 넣는다.
        for history in (reference,unchecked,adapter.history): history.wheels.add(t,.35)
        raw.append(measured); truth.append(true_rate)
    query=np.arange(209)/208
    expected,_=reference.poses(query,1.)
    accepted,_=adapter.history.poses(query,1.)
    unfiltered,_=unchecked.poses(query,1.)
    error=accepted[:,2]-expected[:,2]; old_error=unfiltered[:,2]-expected[:,2]
    return dict(case=case,samples=len(query),accepted=len(adapter.history.gyro.times),
                rejected=dict(adapter.gyro_rejections),
                input_error_peak_rad_s=float(np.max(np.abs(np.array(raw)-truth))),
                max_yaw_error_rad=float(np.max(np.abs(error))),final_yaw_error_rad=float(error[-1]),
                no_gate_max_yaw_error_rad=float(np.max(np.abs(old_error))),
                no_gate_final_yaw_error_rad=float(old_error[-1]))


if __name__=='__main__':
    files=[ROOT/'src/arena_autonomy/arena_autonomy/lidar_motion_ros.py',
           ROOT/'src/arena_autonomy/arena_autonomy/lidar_motion.py',Path(__file__),
           ROOT/'tests/test_lidar_motion_ros.py',BASE/'perturb.py']
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    results=[probe(c) for c in ('baseline','residual','two_spikes','vibration')]
    print(json.dumps(results),flush=True)
    # ROS sec+nsec와 참조 float의 산술 표현 차이만 허용한다.
    assert results[0]['rejected']=={} and results[0]['max_yaw_error_rad']<1e-12
    assert results[2]['rejected'].get('gyro_slew')==2
    report=dict(scope='208 Hz ROS callback functions, synthetic 1+sin(2*pi*t) rad/s turn; no transport or vehicle physics',
                assumptions=dict(residual_sigma=.02,residual_bias_dps=1.,spike_rad_s=4.,
                                 vibration_amplitude_rad_s=.3,vibration_hz=30.),
                source_sha256=hashes,results=results)
    with (BASE/'gyro_gate_probe_v1.json').open('x',encoding='utf-8') as stream:
        json.dump(report,stream,ensure_ascii=False,indent=2)
