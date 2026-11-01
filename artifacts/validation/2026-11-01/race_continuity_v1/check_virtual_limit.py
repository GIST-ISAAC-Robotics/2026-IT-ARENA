"""고속 명령을 가상 MCU와 최종 수신기가 같은 한도로 검사하며 STOP은 그대로인지 확인."""
import json
from pathlib import Path
import runpy
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig
from arena_vehicle_interface.applied_guard import AppliedGuard

here=Path(__file__).resolve().parent
root=here.parents[3]
limit_for=runpy.run_path(str(root/'src/arena_bringup/launch/simulation.launch.py'))['virtual_command_limit']


def check(profile, target):
    limit=limit_for(profile)
    cfg=ContractConfig(wheel_radius_m=.0325,ticks_per_rev=28,gear_ratio=2.5*61/17,
                       feedback_mode='drive_motor_shaft',max_speed_mps=limit)
    rx,tx=ActuationReceiver(cfg,boot_id='compare'),CommandProducer(client_id='host')
    out=AppliedGuard(max_speed=limit)
    for i in range(24):
        t=i*10000
        rx.update_motor_encoder(t,sequence=i,capture_us=t,motor_counts=0);rx.tick(t)
    assert rx.receive(tx.control(rx.offer(t),'ARM'),t)[0]
    assert out.update(dict(rx.status(),now_us=t),t,t*1e-6)
    t+=1000
    packet=tx.drive(rx.offer(t),speed_mps=target,steering_rad=0.,source_us=t,host_now_us=t)
    accepted=rx.receive(packet,t)[0]
    out.update(dict(rx.status(),now_us=t),t,t*1e-6)
    transmitted=out.speed
    t+=1000; assert rx.receive({'v':1,'kind':'STOP'},t)[0]
    out.update(dict(rx.status(),now_us=t),t,t*1e-6)
    assert (accepted,transmitted)==((True,target) if target<=limit else (False,0.))
    assert out.speed==0.
    return dict(profile=profile,limit_mps=limit,request_mps=target,accepted=accepted,applied_mps=transmitted,stop_mps=out.speed)


if __name__=='__main__':
    rows=[check(p,t) for p,t in [('g1_open_2p0',2.),('g1_open_2p0',10/3.6),
                                 ('g1_open_10kmh',10/3.6),('g1_open_10kmh',2.8)]]
    payload=json.dumps({'scope':'host, virtual receiver and final input contract; not dynamics or real firmware','rows':rows},indent=2)
    with (here/'virtual_limit_results.json').open('x') as f:f.write(payload+'\n')
    print(payload)
