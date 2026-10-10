"""평가 전용 플러그인 명령 수신/적용 이벤트: 순수 헤더 동작·launch 기본 불변·분석기 순수 함수."""
from array import array
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
spec = importlib.util.spec_from_file_location('trace_launch', ROOT/'src/arena_bringup/launch/simulation.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)

import analyze_apply_trace as apply_analysis  # noqa: E402
import analyze_evaluator_timing as timing_analysis  # noqa: E402

MS = 1_000_000
HARNESS = r'''
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include "arena_gazebo/command_trace.hpp"
using namespace arena;
struct Plugin {
  ChannelReceiveTrace speedRx, steeringRx; CommandApplyTrace apply; std::vector<std::string> events;
  uint64_t lastIteration{}; int64_t lastSim{-1}; double speed{}, steering{}; bool stale{true};
  void Speed(double v, int64_t wall) { speed = v; speedRx.Received(v, wall, lastIteration, lastSim); stale = false; }
  void Steer(double v, int64_t wall) { steering = v; steeringRx.Received(v, wall, lastIteration, lastSim); }
  void Update(uint64_t iteration, bool forceStale = false) {
    const int64_t sim = static_cast<int64_t>(iteration) * 1000000;
    const bool s = forceStale || stale;
    auto a = speedRx.Take(); auto b = steeringRx.Take();
    lastIteration = iteration; lastSim = sim;
    apply.Step({iteration, sim, 1000000, sim / 1000 + 7}, s ? 0 : speed, s ? 0 : steering, s, a, b, events);
  }
};
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  const std::string test = argv[1];
  Plugin p;
  if (test == "scenario") {
    p.Update(1);                       // first_update (감시 만료 상태, 0 적용)
    p.Speed(0.0, 10); p.Steer(0.0, 11); // ARM 뒤 0 명령: 적용값 변화 없음 → 이벤트 없음
    for (uint64_t i = 2; i <= 5; ++i) p.Update(i);
    p.Speed(0.5, 50); p.Speed(0.5, 51); // 같은 값 두 번 → 첫 메시지 번호 유지
    p.Update(6);
    p.Steer(0.1, 60);
    p.Update(7);
    p.Speed(0.7, 70); p.Speed(0.8, 71); // 0.7은 어떤 PreUpdate에도 보이지 않고 덮임
    p.Update(8);
    p.Speed(0.9, 80); p.Speed(0.8, 81); // 적용값 그대로(0.8) → 이벤트 없음, 덮인 전환 1 추가
    p.Update(9);
    for (uint64_t i = 10; i <= 1000; ++i) p.Update(i);  // 1 s 경계 요약
    p.Update(1001, true);               // 감시 만료 → watchdog 원인 0 적용
    for (const auto &e : p.events) std::cout << e << "\n";
  } else if (test == "bench") {
    p.Update(1); p.Speed(0.4, 1); p.Steer(0.1, 1); p.Update(2);
    const auto begin = std::chrono::steady_clock::now();
    const uint64_t n = 2000000;
    for (uint64_t i = 3; i < n + 3; ++i) { p.speedRx.Received(0.4, 1, i - 1, 0); p.Update(i); }
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begin).count();
    std::cout << "{\"ns_per_step\":" << static_cast<double>(ns) / n << ",\"events\":" << p.events.size() << "}\n";
  } else return 2;
}
'''


@pytest.fixture(scope='module')
def harness(tmp_path_factory):
    temp = tmp_path_factory.mktemp('command_trace_cpp')
    source = temp/'harness.cpp'
    source.write_text(HARNESS)
    exe = temp/'harness'
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
                    '-I', str(ROOT/'src/arena_gazebo/include'), str(source), '-o', str(exe)],
                   check=True, capture_output=True, text=True)
    return exe


def events(harness, name):
    output = subprocess.run([str(harness), name], check=True, capture_output=True, text=True).stdout
    return [json.loads(line) for line in output.splitlines()]


def test_trace_scenario(harness):
    out = events(harness, 'scenario')
    assert [e['event_seq'] for e in out] == list(range(1, len(out)+1))
    assert out[0]['type'] == 'first_update' and out[0]['iteration'] == 1 and out[0]['stale'] is True
    applies = [e for e in out if e['type'] == 'apply']
    speed = [e for e in applies if e['channel'] == 'speed']
    steer = [e for e in applies if e['channel'] == 'steering']
    first, second = speed[0], speed[1]
    assert (first['from'], first['to'], first['iteration'], first['cause']) == (0, .5, 6, 'command')
    assert (first['rx_seq_first'], first['rx_seq_latest'], first['steps_after_rx']) == (2, 3, 1)
    assert first['rx_after_iteration'] == 5 and first['rx_after_sim_ns'] == 5*MS and first['rx_wall_ns'] == 50
    assert first['rx_to_apply_wall_ns'] == 6*MS//1000+7-50
    assert (second['from'], second['to'], second['iteration'], second['rx_seq_first']) == (.5, .8, 8, 5)
    assert second['rx_superseded_total'] == 1
    assert (steer[0]['to'], steer[0]['iteration'], steer[0]['steps_after_rx']) == (.1, 7, 1)
    assert speed[-1]['cause'] == 'watchdog' and speed[-1]['to'] == 0 and steer[-1]['cause'] == 'watchdog'
    assert len(speed) == 3 and len(steer) == 2  # 적용값이 같은 9번째 step은 이벤트가 없다
    summary = [e for e in out if e['type'] == 'summary']
    assert len(summary) == 1 and summary[0]['sim_ns'] == 1_000*MS
    assert summary[0]['speed']['rx_messages'] == 7 and summary[0]['speed']['rx_superseded'] == 2
    assert summary[0]['speed']['multi_message_steps'] == 3 and summary[0]['speed']['apply_events'] == 2


def test_trace_overhead_bound(harness):
    result = events(harness, 'bench')[0]
    # 변화 없는 step의 추적 비용 상한(1 ms 물리 step 대비). 실측값은 산출물에 따로 기록한다.
    # first_update + 첫 전환 2개 + 1 s sim마다 요약 2000개(2,000,002 step × 1 ms).
    assert result['events'] == 3+2000 and result['ns_per_step'] < 5_000


MODEL = '<sdf><model name="car"><link name="body"/><plugin name="arena::SingleMotorDrive"><gear_ratio>15</gear_ratio></plugin></model></sdf>'


def test_launch_trace_default_identical_and_explicit_only():
    assert launch._evaluation_command_trace(MODEL, False, '') == MODEL
    assert launch._evaluation_command_trace(MODEL, False, '/tmp/evidence') == MODEL
    changed = launch._evaluation_command_trace(MODEL, True, '/tmp/evidence')
    tree = ET.fromstring(changed)
    plugin = tree.find('model/plugin')
    node = plugin.find('evaluation_command_trace')
    assert node.text == 'true'
    plugin.remove(node)
    assert ET.tostring(tree) == ET.tostring(ET.fromstring(MODEL))
    with pytest.raises(ValueError):
        launch._evaluation_command_trace(MODEL, True, '')
    with pytest.raises(ValueError):
        launch._evaluation_command_trace(changed, True, '/tmp/evidence')
    with pytest.raises(ValueError):
        launch._evaluation_command_trace('<sdf><model/></sdf>', True, '/tmp/evidence')


def test_launch_trace_argument_default_false():
    description = launch.generate_launch_description()
    arguments = {a.name: a for a in description.entities if type(a).__name__ == 'DeclareLaunchArgument'}
    assert arguments['evaluation_command_trace'].default_value[0].text == 'false'
    assert set(arguments['evaluation_command_trace'].choices) == {'true', 'false'}


def test_plugin_trace_branches_are_guarded():
    """플러그인 추적 호출은 모두 traceEnabled 분기 안에 있고 값 샘플 문장은 그대로다."""
    text = (ROOT/'src/arena_gazebo/src/single_motor_drive.cpp').read_text(encoding='utf-8')
    for call in ('speedRx.Received', 'steeringRx.Received', 'applyTrace.Step', 'applyTrace.Reset', 'traceTopic.Publish'):
        for line in [l for l in text.splitlines() if call in l]:
            index = text.splitlines().index(line)
            window = '\n'.join(text.splitlines()[max(0, index-8):index+1])
            assert 'traceEnabled' in window, call
    assert 'speed = stale ? 0 : commandSpeed;' in text and 'steering = stale ? 0 : commandSteering;' in text
    assert 'traceEnabled = sdf->Get<bool>("evaluation_command_trace", false).first;' in text


# ---- 분석기 순수 함수 -------------------------------------------------------------------

def diag(t, **values):
    row = {k: 0. for k in apply_analysis.DIAG_KEYS}
    row.update(sim_time_s=t, **values)
    return row


def test_divergence_first_difference_and_levels():
    a = [diag(i*.02) for i in range(10)]
    b = [diag(i*.02, left_speed_rad_s=(1e-8 if i >= 4 else 0.)+(1e-2 if i >= 7 else 0.)) for i in range(10)]
    result = apply_analysis.divergence(a, b, 0)
    assert result['first_any_difference_rel_ns'] == 80*MS and result['first_any_difference_keys'] == ['left_speed_rad_s']
    assert result['first_wheel_or_body_speed_exceeds_rel_ns']['1e-09'] == 80*MS
    assert result['first_wheel_or_body_speed_exceeds_rel_ns']['0.001'] == 140*MS
    assert result['first_wheel_or_body_speed_exceeds_rel_ns']['0.1'] is None
    assert apply_analysis.divergence(a, a, 0)['first_any_difference_rel_ns'] is None


def trace(applies):
    return {channel: [{'to': to, 'apply_rel_ns': t, 'steps_after_rx': 1} for to, t in applies.get(channel, [])]
            for channel in ('speed', 'steering')}


@pytest.mark.parametrize('spawn_same,a,b,first_div,label', [
    (False, {'speed': [(.1, 5*MS)]}, {'speed': [(.1, 5*MS)]}, None, 'spawn_differs_initial_state_not_identical'),
    (True, {'speed': [(.1, 5*MS)]}, {'speed': [(.1, 5*MS)]}, None,
     'reproduced_at_print_precision_given_same_spawn_and_applications'),
    (True, {'speed': [(.1, 5*MS)]}, {'speed': [(.1, 5*MS)]}, 40*MS, 'diverged_without_traced_application_difference'),
    (True, {'speed': [(.1, 5*MS), (.2, 30*MS)]}, {'speed': [(.1, 5*MS), (.2, 31*MS)]}, 40*MS,
     'divergence_onset_not_before_first_application_difference'),
    (True, {'speed': [(.1, 5*MS), (.2, 30*MS)]}, {'speed': [(.1, 5*MS), (.2, 31*MS)]}, 20*MS,
     'divergence_precedes_first_application_difference'),
    (True, {'speed': [(.1, 5*MS), (.2, 30*MS)]}, {'speed': [(.1, 5*MS), (.2, 31*MS)]}, None,
     'application_difference_without_observed_divergence'),
    (True, {'speed': [(.1, 5*MS)]}, {'speed': [(.1, 5*MS), (.2, 9*MS)]}, None, 'applications_not_comparable'),
    (True, {'speed': [(.1, 5*MS)]}, {'speed': [(.3, 5*MS)]}, None, 'applications_not_comparable'),
])
def test_registered_classification(spawn_same, a, b, first_div, label):
    applications = apply_analysis.first_application_difference(trace(a), trace(b))
    assert apply_analysis.classify(spawn_same, applications, {'first_any_difference_rel_ns': first_div}) == label


def test_trace_summary_and_alignment():
    events = [{'type': 'first_update', 'event_seq': 1, 'iteration': 23, 'sim_ns': 23*MS, 'dt_ns': MS},
              {'type': 'apply', 'event_seq': 2, 'channel': 'speed', 'cause': 'command', 'to': .1, 'iteration': 1456,
               'sim_ns': 1456*MS, 'dt_ns': MS, 'rx_after_sim_ns': 1455*MS, 'steps_after_rx': 1, 'rx_to_apply_wall_ns': 9,
               'rx_seq_first': 4, 'rx_seq_latest': 4, 'rx_superseded_total': 0},
              {'type': 'apply', 'event_seq': 3, 'channel': 'steering', 'cause': 'watchdog', 'to': 0., 'iteration': 9000,
               'sim_ns': 9000*MS, 'dt_ns': MS, 'rx_after_sim_ns': 1, 'steps_after_rx': 1, 'rx_to_apply_wall_ns': 1,
               'rx_seq_first': 1, 'rx_seq_latest': 1, 'rx_superseded_total': 0}]
    summary = apply_analysis.trace_summary(events, 1_000*MS, 3_000*MS)
    assert summary['spawn'] == {'iteration': 23, 'sim_ns': 23*MS, 'dt_ns': MS}
    assert summary['sim_ns_minus_iteration_dt_ns'] == [0] and summary['watchdog_applies'] == 1
    assert summary['speed'][0]['apply_rel_ns'] == 456*MS and summary['steering'] == []
    stages = {'speed': {'events': 1, 'stages_rel_start_ns': {'mcu': [450*MS], 'output': [455*MS]}},
              'steering': {'events': 1, 'stages_rel_start_ns': {'mcu': [350*MS]}}}
    aligned = apply_analysis.stage_alignment(summary, stages)
    assert aligned['speed']['rows'][0]['apply_minus_output_ns'] == MS
    assert aligned['speed']['rows'][0]['rx_after_minus_output_ns'] == 0
    assert aligned['steering'] == {'matched': False, 'applies': 0, 'expected': 1}


def write_timing(run, report, poll_clock, poll_wall, own_sim, own_wall):
    arrays = {'poll_wall_before_spin_ns': [w-100 for w in poll_wall], 'poll_wall_after_spin_ns': poll_wall,
              'poll_node_clock_ns': poll_clock, 'clock_msg_sim_ns': own_sim, 'clock_msg_wall_ns': own_wall}
    with (run/'evaluator_timing.i64').open('wb') as stream:
        for name in timing_analysis.TIMING_ARRAYS:
            array('q', arrays[name]).tofile(stream)
    report['evaluator_timing'] = {'path': 'evaluator_timing.i64',
                                  'arrays_in_order': {k: len(v) for k, v in arrays.items()}, 'callback_stats': {}}
    (run/'report.json').write_text(json.dumps(report))


def test_timing_categories(tmp_path):
    report = {'command_schedule': {'start_ns': 0}}
    # 1 ms 시계, RTF 0.1(1 ms sim = 10 ms 벽시계). tick 0 정상, tick 1(20 ms)은 노드 시계가 14 ms에서 멈췄다가 32 ms로 도약.
    clock = [0, 1*MS, 2*MS, 14*MS, 14*MS, 14*MS, 14*MS, 32*MS, 40*MS, 41*MS]
    wall = [0, 10*MS, 20*MS, 140*MS, 150*MS, 260*MS, 300*MS, 320*MS, 400*MS, 410*MS]
    own_sim = [0, MS, 2*MS, 14*MS, 32*MS]
    own_wall = [0, 10*MS, 20*MS, 140*MS, 319*MS]
    write_timing(tmp_path, report, clock, wall, own_sim, own_wall)
    rows = [{'index': 0, 'scheduled_ns': 0, 'observed_ns': 0}, {'index': 1, 'scheduled_ns': 20*MS, 'observed_ns': 32*MS}]
    (tmp_path/'requested_commands.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    result = timing_analysis.analyze(tmp_path)
    tick = result['late_ticks'][0]
    assert tick['lateness_ns'] == 12*MS and tick['node_clock_jump_ns'] == 18*MS and tick['frozen_polls_before_jump'] == 3
    assert tick['category'] == 'node_clock_frozen_while_polling:own_clock_receipts_also_paused'
    # 같은 도약을 한 번의 긴 poll 간격으로 바꾸면 단일 정체로 분류된다.
    clock2 = [0, 1*MS, 2*MS, 14*MS, 32*MS, 40*MS]
    wall2 = [0, 10*MS, 20*MS, 140*MS, 320*MS, 400*MS]
    run2 = tmp_path/'b'
    run2.mkdir()
    write_timing(run2, dict(report), clock2, wall2, own_sim, own_wall)
    (run2/'requested_commands.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    assert timing_analysis.analyze(run2)['late_ticks'][0]['category'] == 'single_poll_wall_stall'


def test_timing_rejects_length_mismatch(tmp_path):
    report = {'command_schedule': {'start_ns': 0}}
    write_timing(tmp_path, report, [0, MS], [0, 10*MS], [], [])
    report['evaluator_timing']['arrays_in_order']['poll_node_clock_ns'] = 1
    with pytest.raises(ValueError):
        timing_analysis.load_timing(tmp_path, report)
