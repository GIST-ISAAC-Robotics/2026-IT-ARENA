"""r221의 느린 현재 옆차 검사를 프로파일링한다. 장치/ROS/발행 없음."""
from pathlib import Path
import cProfile, hashlib, io, json, pstats, sys
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, equal_output, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy import guard_observation_policy as policy

A = Path(__file__).resolve().parent
source = Path(sys.argv[1]).resolve()
root = source.parents[3]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
samples = source/'samples.jsonl'
before = {p.relative_to(root).as_posix():sha(p) for p in (root/'src').rglob('*.py')}
inputs = sha(samples)
rows = [json.loads(line) for line in samples.open()]
owner, motion = make_controller(rows[0]['header'])
selected = [row for row in rows[1:] if row['expected']['reason'] == 'rebase_current_present_guard']
prepared = []
for row in selected:
    data, scan, history = restore_call(row['current'], motion)
    scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                         epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
    prepared.append((row, data, scene, history))
profiler = cProfile.Profile()
answers = []
for row, data, scene, history in prepared:
    answer = profiler.runcall(policy.rebase, row['original']['output'], row['original']['input'],
                              row['current'], scene, history, owner.config)
    keys = ('speed_mps', 'steering_rad', 'reason', 'reference_s')
    answers.append(dict(seq=row['seq'], full_equal=equal_output(freeze(answer), row['expected']),
        command_equal=equal_output([answer.get(k) for k in keys], [row['expected'].get(k) for k in keys])))
profiler.dump_stats(str(A/'recheck.prof'))
stream = io.StringIO()
pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats('cumulative').print_stats(35)
(A/'cumulative.txt').write_text(stream.getvalue())
stream = io.StringIO()
pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats('tottime').print_stats(25)
(A/'internal.txt').write_text(stream.getvalue())
after = {p.relative_to(root).as_posix():sha(p) for p in (root/'src').rglob('*.py')}
report = dict(cases=len(answers), rows=answers, sources_unchanged=before==after,
              input_unchanged=inputs==sha(samples), source_sha256=before, input_sha256=inputs,
              scope='cProfile of current present-guard recheck only; timings instrumented, not latency benchmark')
with (A/'report.json').open('x') as out:
    json.dump(report,out,indent=2);out.write('\n')
print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'}),flush=True)
raise SystemExit(0 if report['sources_unchanged'] and report['input_unchanged'] and all(r['command_equal'] for r in answers) else 1)
