"""race_ops_eval·summarize_motion_lap 수정 사본 생성(원본 줄끝 유지). patched/scripts의 사본만 바꾼다."""
from pathlib import Path

HERE = Path(__file__).resolve().parent/'patched/scripts'


def edit(name, pairs):
    p = HERE/name
    t = p.read_bytes().decode('utf-8')
    crlf = '\r\n' in t
    t = t.replace('\r\n', '\n')
    for old, new in pairs:
        assert t.count(old) == 1, (name, old[:60], t.count(old))
        t = t.replace(old, new)
    p.write_bytes((t.replace('\n', '\r\n') if crlf else t).encode('utf-8'))


edit('race_ops_eval.py', [
("""SOURCES = ('race_session', 'local_pursuit', 'wall_follow', 'vision_process', 'lidar_motion_ros', 'lidar_safety',
           'race_ops_core', 'race_ops_node', 'race_ops_export', 'race_ops_page')
""", """SOURCES = ('race_session', 'local_pursuit', 'wall_follow', 'vision_process', 'lidar_motion_ros', 'lidar_safety',
           'race_ops_core', 'race_ops_node', 'race_ops_export', 'race_ops_page')
# 정지·재배치·상대 s좌표와 ID30·섬 통과 판정을 등록한 track. 다른 track은 재등록 전 실행하지 않는다.
REGISTERED_SCENARIOS = {'official': ('representative', 'across_marker', 'parked_recovery', 'browser_stop')}
"""),
("""class Course:
    def __init__(self):
        route = np.genfromtxt(WORLD/'centerline.csv', delimiter=',', names=True)
        self.xy = np.column_stack((route['x_m'], route['y_m']))
        self.s = route['s_m']
        self.branch = np.genfromtxt(WORLD/'branch_1.csv', delimiter=',', skip_header=1, usecols=(0, 1))
""", """class Course:
    def __init__(self, world=WORLD):
        route = np.genfromtxt(world/'centerline.csv', delimiter=',', names=True)
        self.xy = np.column_stack((route['x_m'], route['y_m']))
        self.s = route['s_m']
        self.branch = np.genfromtxt(world/'branch_1.csv', delimiter=',', skip_header=1, usecols=(0, 1))
"""),
("""def main():
    parser = argparse.ArgumentParser(description=__doc__)
""", """def main():
    from validate_basic_autonomy import DEMO_TRACK, TRACK_WORLDS
    parser = argparse.ArgumentParser(description=__doc__)
"""),
("""    args = parser.parse_args()
    if args.browser_stop and args.scenario != 'parked_recovery':
        parser.error('--browser-stop requires parked_recovery')
""", """    parser.add_argument('--track', choices=tuple(TRACK_WORLDS), default=DEMO_TRACK,
                        help='실행·평가 월드. 운영 시나리오를 등록한 track만 실행한다.')
    args = parser.parse_args()
    if args.browser_stop and args.scenario != 'parked_recovery':
        parser.error('--browser-stop requires parked_recovery')
    if args.scenario not in REGISTERED_SCENARIOS.get(args.track, ()):
        parser.error(f'{args.track}의 {args.scenario} 운영 시나리오는 미등록이다. 옛 정지·재배치·상대 s좌표를 옮겨 쓰지 않고 재등록한다')
    world = TRACK_WORLDS[args.track]
"""),
("""    sources['scripts/race_ops_eval.py'] = digest(__file__)
""", """    sources['scripts/race_ops_eval.py'] = digest(__file__)
    if args.track != DEMO_TRACK:
        for name in ('world.sdf', 'scene.json', 'centerline.csv', 'branch_1.csv'):
            sources[(world/name).relative_to(REPO).as_posix()] = digest(world/name)
"""),
("""    report = dict(scenario=args.scenario, started_wall=time.time(), source_sha256=sources, passed=False, steps=[])
""", """    report = dict(scenario=args.scenario, started_wall=time.time(), source_sha256=sources, passed=False, steps=[])
    if args.track != DEMO_TRACK:
        report['track'] = args.track
"""),
("""              'g1_standing_wall_reacquire:=true']
""", """              'g1_standing_wall_reacquire:=true']
    if args.track != DEMO_TRACK:
        extra.append('track:='+args.track)
"""),
("""        actor_pose = Course().pose(2.1)
""", """        actor_pose = Course(world).pose(2.1)
"""),
("""        course = Course()
""", """        course = Course(world)
"""),
])

edit('summarize_motion_lap.py', [
("""import json
from pathlib import Path
""", """import json
from pathlib import Path
import sys
"""),
("""REPO = Path(__file__).resolve().parents[1]
""", """REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
from validate_basic_autonomy import DEMO_TRACK, TRACK_WORLDS
"""),
("""    parser.add_argument("run", type=Path)
    args = parser.parse_args()
""", """    parser.add_argument("run", type=Path)
    parser.add_argument("--track", choices=tuple(TRACK_WORLDS), default=DEMO_TRACK,
                        help="궤적 겹침용 평가 중심선 월드. report.json의 track과 같아야 합니다.")
    args = parser.parse_args()
"""),
("""    report = json.loads((folder / "report.json").read_text())
""", """    report = json.loads((folder / "report.json").read_text())
    recorded = report.get("track", DEMO_TRACK)
    if recorded in TRACK_WORLDS and recorded != args.track:
        raise ValueError(f"report.json track={recorded}와 --track={args.track}이 다릅니다.")
"""),
("""              "scope": "sampled simulator truth used only for evaluation; longitudinal speed is not commanded or wheel speed"}
""", """              "scope": "sampled simulator truth used only for evaluation; longitudinal speed is not commanded or wheel speed"}
    if args.track != DEMO_TRACK:
        result["track"] = args.track
"""),
("""    route = np.genfromtxt(REPO / "src/arena_gazebo/worlds/it_arena_official/centerline.csv", delimiter=",", names=True)
""", """    route = np.genfromtxt(TRACK_WORLDS[args.track] / "centerline.csv", delimiter=",", names=True)
"""),
("""    ax[0].set_title(f"Official course / completed laps: {result['completed_laps']}")
""", """    course = "Official course" if args.track == DEMO_TRACK else f"Official course {args.track}"
    ax[0].set_title(f"{course} / completed laps: {result['completed_laps']}")
"""),
])
print('ok')
