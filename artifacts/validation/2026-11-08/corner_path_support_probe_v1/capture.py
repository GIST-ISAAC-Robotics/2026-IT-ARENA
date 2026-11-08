"""r30 실패 원본을 당시 소스로 재생하며 현재 벽과 경로 기억을 분리한다."""
from pathlib import Path
import hashlib, json, sys, tempfile, zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUN = HERE.parent/'fragment_boundary_connected_v1/right_parallel_first_curve'
TRACE = RUN/'run1/follow_decisions.jsonl'
registry = json.loads((RUN/'registered.json').read_bytes())
snapshot = RUN/'source_snapshot.zip'
assert hashlib.sha256(snapshot.read_bytes()).hexdigest() == registry['snapshot_sha256']
temporary = tempfile.TemporaryDirectory(prefix='r30_boundary_')
tree = Path(temporary.name)
with zipfile.ZipFile(snapshot) as archive:
    for name in archive.namelist():
        if name.startswith('src/') and name.endswith('.py'):
            dest = tree/name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(archive.read(name))
for package in (tree/'src').iterdir():
    if package.is_dir():
        sys.path.insert(0, str(package))

from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow, transport_points
import arena_autonomy.local_follow as lf
assert str(tree) in lf.__file__

captures = []
original = LocalFollow._baseline_command

def instrument(self, scene, history, *args, **kw):
    now = scene.t_ref_s
    record = None
    if 31.9 <= now <= 38.1 and (self.last_source is None or scene.source_first_s > self.last_source):
        carried = None
        if self.path is not None:
            try:
                carried = transport_points(self.path, history, self.path_ref, now).tolist()
            except ValueError:
                pass
        record = dict(t=now, source=scene.source_first_s, old_path_source=self.path_source,
            old_path_ref=self.path_ref, old_wall=self.path_wall,
            carried=carried, points=scene.occupied_endpoints_xy_m,
            speed=args[0], side=args[1], previous=args[2], config=self.config.__dict__)
    out = original(self, scene, history, *args, **kw)
    if record is not None:
        record['baseline'] = out
        captures.append(freeze(record))
    return out

LocalFollow._baseline_command = instrument
matched = resets = 0
for line in TRACE.open(encoding='utf8'):
    event = json.loads(line)
    data = event['data']
    if event['kind'] == 'header':
        header = data
        motion = MotionConfig(**data['motion_config'])
        engine = BypassHandoffFollow(FollowConfig(**data['follow_config']),
            HandoffExecutionConfig.from_mapping(data['execution_config']), allow_simulation_trial=True)
    elif event['kind'] == 'reset':
        engine.reset(); resets += 1
    elif event['kind'] == 'call':
        now = data['input']['reference_s']
        if now > 38.1:
            break
        actual = freeze(replay_call(engine, data['input'], motion))
        assert equal_output(comparable_decision(actual), comparable_decision(data['output'])), now
        matched += 1

(HERE/'snapshots.json').write_text(json.dumps(dict(header=header,captures=captures),
    ensure_ascii=False)+'\n', encoding='utf8')
brief = [dict(t=r['t'], source=r['source'], old_source=r['old_path_source'],
    carried=r['carried'] is not None, output=r['baseline'][:2],
    reason=r['baseline'][2].get('reason'), selection=r['baseline'][2].get('boundary_selection'),
    fresh_accepted=r['baseline'][2].get('fresh_boundary_accepted')) for r in captures]
result = dict(matched=matched,resets=resets,trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
    snapshot_sha256=registry['snapshot_sha256'],scope='r30 historical exact replay, no simulation or runtime edits',rows=brief)
(HERE/'capture_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(matched=matched,resets=resets,captures=len(captures))),flush=True)
