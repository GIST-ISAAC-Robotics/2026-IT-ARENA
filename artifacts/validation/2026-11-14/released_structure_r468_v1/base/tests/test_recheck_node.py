"""별도 현재 검사 결과의 실제 노드 연결. 계산/발행 시각과 원문 증거를 분리한다."""
import copy
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
from arena_autonomy.g1_g2_node import G1G2Node
from arena_autonomy.follow_trace import equal_output
from test_current_path_recheck_node import records, make_node


def test_worker_current_result_is_used_once_without_parent_recalculation(records):
    header, calls = records
    call = next(c for c in calls if 'current_rebase' in c)
    node, trace, _ = make_node(header, call, False)
    node.recheck_in_worker = True
    current = copy.deepcopy(call['current_rebase'])
    item = node.decision_delivery.poll()
    item['recheck_input'] = current['input']
    item['recheck_result'] = dict(output=current['output'], began_wall_s=1., completed_wall_s=1.01,
                                  stage='recheck', generation=0, sequence=2)
    out = current['output']
    item['output'] = (out['speed_mps'], out['steering_rad'],
        {**item['output'][2], 'current_rebase': {k:v for k,v in out.items() if k != 'path_points'}})
    item['command_reference_s'] = out['reference_s']
    node.decision_delivery.poll = lambda *a, **kw:item
    node.current_path_recheck = NS(supported=lambda _:pytest.fail('parent recomputed worker result'))
    result = G1G2Node.command(node, [])
    assert result[:2] == pytest.approx((out['speed_mps'], out['steering_rad']))
    assert node.decision_command_stamp_s == out['reference_s']
    evidence = trace[-1][1]
    assert equal_output(evidence['input'], call['input'])
    assert equal_output(evidence['output'], call['output'])
    assert equal_output(evidence['current_rebase']['input'], current['input'])
    assert equal_output(evidence['current_rebase']['output'], out)


def test_plan_completion_only_submits_current_input_and_does_not_issue_command(records):
    header, calls = records
    call = next(c for c in calls if 'current_rebase' in c)
    node, trace, _ = make_node(header, call, False)
    sent = []
    node.decision_delivery.poll = lambda *a, **kw:dict(kind='needs_recheck')
    node.decision_delivery.offer_recheck = sent.append
    assert G1G2Node.command(node, []) is None
    assert node.decision_command_stamp_s is None and trace == []
    assert len(sent) == 1
    assert sent[0]['reference_s'] == node.motion.last_meta['reference_stamp_s']
    assert sent[0]['previous_steering_rad'] == node.last_steering
    assert sent[0]['dt_s'] == node.decision_output_dt_s


@pytest.mark.parametrize('fault', [None, 'child exited'])
def test_notice_collects_before_normal_control_and_acknowledges(fault):
    events = []
    notice = NS(close=lambda:events.append('close'), acknowledge=lambda:events.append('ack'))
    node = NS(decision_notice=notice,
        decision_delivery=NS(collect=lambda:events.append('collect'), worker=NS(fault=fault)),
        control=lambda:events.append('full_control'))
    G1G2Node.on_decision_ready(node)
    assert events == ['collect', *(['close'] if fault else []), 'full_control', 'ack']
    assert (node.decision_notice is None) == bool(fault)


def test_notice_acknowledged_even_if_control_raises():
    events=[]
    def fail():raise RuntimeError('control failure')
    node=NS(decision_notice=NS(acknowledge=lambda:events.append('ack')),
        decision_delivery=NS(collect=lambda:None,worker=NS(fault=None)),control=fail)
    with pytest.raises(RuntimeError,match='control failure'):G1G2Node.on_decision_ready(node)
    assert events == ['ack']
