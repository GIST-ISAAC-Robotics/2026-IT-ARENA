"""JR60P20의 이동 앞차 장면 등록. 제어기에는 상대 각본을 전달하지 않는다."""
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = ROOT / 'config/tests/official_forward_v1.json'
SENSOR = ROOT / 'src/arena_description/config/b0183_c1_jr60p20_sim.yaml'
GRID = ROOT / 'src/arena_gazebo/worlds/it_arena_official/scene.json'
CASE = 'straight_moving_lead'


def dump(value):
    return (json.dumps(value, indent=2, ensure_ascii=False) + '\n').encode('utf-8')


def prepare():
    cfg = json.loads(BASE.read_bytes())
    cfg.update(registered_at='2026-11-05', scope='JR60P20 Ø20 배선·ID49 140 mm. 직선 등속차 추종 기준. 이동 추월 제어 OFF.',
               parent_config={'path': BASE.relative_to(ROOT).as_posix()})
    cfg['actor']['visible_section'] = '높이 0–120 mm의 각본 상자를 75 mm C1 평면이 가로지른다. 자유 주행 상대가 아니다.'
    cfg['cases'] = [dict(name=CASE, behavior_kind='slow_lead', route_kind='straight', duration_s=14.,
                         schedule=[dict(from_s=0., to_s=14., speed_mps=.30)],
                         expected='전 구간에서 움직이는 앞차. 추종 기준 실행에서 기존 행동을 측정한다.')]
    cfg['acceptance'][CASE] = dict(cfg['acceptance']['slow_lead'])
    cfg['matrix'] = {'g1g2': {'cases': [CASE], 'seeds': [0]}}
    scene = json.loads(GRID.read_bytes())
    s0 = next(s['s_m'] for s in scene['starting_grid']['slots'] if s['index']==cfg['launch']['grid_slot']) + cfg['actor']['start_center_gap_m']
    lap = scene['track']['lap_length_m']
    corner = cfg['first_corner_s_m']
    while corner < s0:
        corner += lap
    assert corner-(s0+.30*14.) >= 1. and cfg['launch']['speed_profile']=='g1_open_1p5'
    h=lambda b:hashlib.sha256(b).hexdigest()
    cfg_bytes=dump(cfg)
    args=['scripts/validate_g1_traffic.py','--mode','g1g2','--handoff-reserve','paced','--handoff-yield-progress',
          '--handoff-rear-sweep','--official-signal-course','--config',str(HERE/'course.json'),'--case',CASE,
          '--seed','0','--record-follow-decisions','--handoff-observed-motion','--side-departure-trial',
          '--current-lane-preference-trial','--local-sensor-config',str(SENSOR),'--pilot-marker-z','0.140',
          '--output',str(HERE/'run1')]
    # 플랫폼별 절대 경로는 실제 호출할 때만 만든다. 등록 파일은 Windows/WSL에서 동일하다.
    manifest=dict(course_sha256=h(cfg_bytes),parent_sha256=h(BASE.read_bytes()),sensor_sha256=h(SENSOR.read_bytes()),
                  setup_sha256=h(Path(__file__).read_bytes()),scene_sha256=h(GRID.read_bytes()),corner_clearance_m=corner-s0-.30*14.,
                  args=[Path(a).relative_to(ROOT).as_posix() if Path(a).is_absolute() else a for a in args])
    return cfg_bytes,dump(manifest),args


if __name__=='__main__':
    config,manifest,args=prepare()
    files=[(HERE/'course.json',config),(HERE/'registered.json',manifest)]
    if sys.argv[1:]==['--register']:
        if any(p.exists() for p,_ in files):raise SystemExit('registration already exists')
        for p,b in files:
            with p.open('xb') as f:f.write(b)
    elif sys.argv[1:]==['--run']:
        import subprocess
        if any(p.read_bytes()!=b for p,b in files):raise SystemExit('registration changed')
        if (HERE/'run1').exists():raise SystemExit('existing trial output')
        raise SystemExit(subprocess.call([sys.executable,'-B',*args],cwd=ROOT))
    else:raise SystemExit('use --register or --run')
