"""개발 중 비용 분해(개루프 저장 입력 일부). 판정 근거가 아닌 최적화 대상 확인용."""
import cProfile, pstats, sys, io
from dataclasses import replace
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import replay_execution as rx
screen = rx.screen
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig
motion, config, events = screen.load_inputs(screen.RECORDS['e3_onroad_g1'])
config = replace(config, occupancy_timing_mode='per_ray_candidate', recent_observation_enabled=False)
c = M3ExecutionFollow(config, ExecutionConfig(), allow_simulation_trial=True)
calls = [e for e in events if e['kind'] == 'call'][:260]
prepared = []
for e in calls:
    data, scan, history = screen.restore_call(screen.thaw(e['data']['input']), motion)
    scene = screen.motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'], epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
    prepared.append((scene, history, data))
pr = cProfile.Profile(); pr.enable()
for scene, history, data in prepared:
    c.command(scene, history, data['speed_mps'], data['side'], data['previous_steering_rad'], data['dt_s'])
pr.disable()
s = io.StringIO(); pstats.Stats(pr, stream=s).sort_stats('cumulative').print_stats(35); print(s.getvalue()[:7000])
