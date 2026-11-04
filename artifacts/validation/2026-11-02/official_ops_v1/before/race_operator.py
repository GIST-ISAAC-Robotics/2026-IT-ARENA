#!/usr/bin/env python3
"""선택형 B3 사람 운영 도구. 기본은 조회/미전송이며 MCU 명령·실제 정지를 대신하지 않는다."""
import argparse
import json
import math
import time
import uuid

STATUS = '/autonomy/status'
COMMAND = '/race/command'
RESULT = '/race/command_result'
RECEIPT_MAX_AGE = .5  # 도구의 상태 수신 신선도. 센서/MCU 시각 검증을 대신하지 않는다.
BOUNDARY = ('upper race policy only; no MCU STOP/CLEAR/ARM or drive publication; '
            'accepted does not prove physical stop or permission to move')


def token(value):
    return type(value) is str and 1 <= len(value) <= 64 and not any(c.isspace() for c in value)


def natural(value):
    return type(value) is int and value >= 0


def decode(text):
    if not isinstance(text, str) or len(text) > 262144:
        raise ValueError('invalid message size')
    def bad_constant(value):
        raise ValueError('nonfinite JSON')
    def unique(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate JSON key')
            result[key] = value
        return result
    value = json.loads(text, parse_constant=bad_constant, object_pairs_hook=unique)
    if type(value) is not dict:
        raise ValueError('object required')
    return value


def race_status(payload):
    race = payload.get('race')
    if (type(race) is not dict or race.get('mode') != 'b3_candidate' or not token(race.get('session'))
            or not natural(race.get('race_epoch')) or not natural(race.get('intervention'))
            or not (race.get('latch') is None or type(race['latch']) is str)
            or type(race.get('started')) is not bool or type(race.get('rearm_required')) is not bool):
        raise ValueError('valid B3 status required; legacy is not supported')
    return {key: race.get(key) for key in (
        'mode', 'session', 'race_epoch', 'intervention', 'latch', 'started', 'visual_start_fact',
        'policy_allowed_if_sensors_ready', 'policy_reason', 'actuator_reason', 'rearm_required',
        'mcu_stop_observed_during_intervention', 'mcu')}


def build_command(op, *, session=None, race_epoch=None, intervention=None,
                  hazards_cleared=False, operator_reviewed=False, request=None):
    request = uuid.uuid4().hex if request is None else request
    if not token(request):
        raise ValueError('invalid request')
    if op not in ('stop', 'new_race', 'acknowledge'):
        raise ValueError('unsupported operation')
    result = dict(v=1, request=request, op=op)
    if op == 'stop':
        if any(v is not None for v in (session, race_epoch, intervention)) or hazards_cleared or operator_reviewed:
            raise ValueError('stop does not take release tokens')
        return result
    if not token(session) or not natural(race_epoch):
        raise ValueError('explicit session and race epoch required')
    result.update(session=session, race_epoch=race_epoch)
    if op == 'new_race':
        if intervention is not None or hazards_cleared or operator_reviewed:
            raise ValueError('new_race does not take acknowledgement flags')
    else:
        if not natural(intervention) or hazards_cleared is not True or operator_reviewed is not True:
            raise ValueError('explicit intervention, hazards-cleared and operator-reviewed required')
        result.update(intervention=intervention, hazards_cleared=True, operator_reviewed=True)
    return result


def preflight(command, status):
    """확인한 번호를 자동 갱신하지 않는다. 최종 경쟁은 서버의 세대 검사로 거절한다."""
    if command['op'] == 'stop':
        return
    if any(command[key] != status[key] for key in ('session', 'race_epoch')):
        raise ValueError('stale session/race token; read status and review again')
    if command['op'] == 'acknowledge':
        if command['intervention'] != status['intervention'] or status['latch'] is None:
            raise ValueError('stale intervention or no latch; read status and review again')


def matching_result(command, result):
    """관련 없는 응답은 무시한다. 일치하는 잘못된 응답을 성공으로 표시하지 않는다."""
    if result.get('request') != command['request'] or result.get('op') != command['op']:
        return False
    if (type(result.get('v')) is not int or result['v'] != 1 or type(result.get('accepted')) is not bool
            or not token(result.get('session')) or not natural(result.get('race_epoch'))
            or not natural(result.get('intervention')) or type(result.get('reason')) is not str):
        raise ValueError('malformed matching result; outcome unknown')
    if not result['accepted']:
        return True
    op = command['op']
    if op == 'stop':
        valid = result.get('latch') == 'explicit_stop'
    elif op == 'new_race':
        valid = (result['session'] == command['session'] and result['race_epoch'] == command['race_epoch']+1
                 and result.get('latch') == 'new_race')
    else:
        valid = (result['session'] == command['session'] and result['race_epoch'] == command['race_epoch']
                 and result['intervention'] == command['intervention'] and result.get('latch') is None)
    if not valid:
        raise ValueError('inconsistent matching result; outcome unknown')
    return True


def peer_identity(info):
    return (info.node_namespace, info.node_name)


def endpoints_ready(node, *, stop_only=False):
    """운영 인증이 아닌 실수 방지. 같은 이름의 여러 endpoint도 거절한다."""
    endpoints = [node.get_subscriptions_info_by_topic(COMMAND), node.get_publishers_info_by_topic(RESULT)]
    if not stop_only:
        endpoints.append(node.get_publishers_info_by_topic(STATUS))
    if any(len(items) > 1 for items in endpoints):
        raise ValueError('ambiguous B3 endpoints; isolate the ROS domain')
    if not all(endpoints):
        return False
    if len({peer_identity(items[0]) for items in endpoints}) != 1:
        raise ValueError('B3 endpoints belong to different nodes')
    return True


def exchange(op, command=None, *, timeout_s=5.):
    """ROS 신선한 상태 + 명령 1회 + 해당 응답. 시간 초과에 재전송/새 번호 발급하지 않는다."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    if not math.isfinite(timeout_s) or not .1 <= timeout_s <= 30:
        raise ValueError('timeout must be within 0.1..30 seconds')
    context = Context()
    node = executor = None
    latest, received, results = None, None, []
    sent = False
    def on_status(message):
        nonlocal latest, received
        try:
            latest = race_status(decode(message.data))
            received = time.monotonic()
        except ValueError:
            latest, received = None, None
    def on_result(message):
        try:
            data = decode(message.data)
        except ValueError:
            return  # 연관성을 확인할 수 없는 응답으로 성공/실패를 결정하지 않는다.
        try:
            if command is not None and matching_result(command, data):
                results.append(data)
        except ValueError as error:
            results.append(dict(client_error=str(error)))

    try:
        rclpy.init(args=[], context=context)
        node = rclpy.create_node('race_operator_'+uuid.uuid4().hex[:8], context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        node.create_subscription(String, STATUS, on_status, 10)
        if command is not None:
            node.create_subscription(String, RESULT, on_result, 10)
        publisher = None if command is None else node.create_publisher(String, COMMAND, 10)
        deadline = time.monotonic()+timeout_s
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(.05, max(0., deadline-time.monotonic())))
            now = time.monotonic()
            if op == 'status':
                peers = node.get_publishers_info_by_topic(STATUS)
                if len(peers) > 1:
                    raise ValueError('ambiguous status publishers; isolate the ROS domain')
                if len(peers) == 1 and latest is not None and now-received < RECEIPT_MAX_AGE:
                    return 0, dict(outcome='status', race=latest, boundary=BOUNDARY)
                continue
            if not sent:
                if not endpoints_ready(node, stop_only=op == 'stop'):
                    continue
                if op != 'stop':
                    if latest is None or now-received >= RECEIPT_MAX_AGE:
                        continue
                    preflight(command, latest)
                sent = True  # publish가 예외를 내도 실제 배송 여부는 확정할 수 없다.
                publisher.publish(String(data=json.dumps(command, allow_nan=False)))
                results.clear()  # 전송 전 결과는 채택하지 않는다. 콜백은 같은 executor에서 돈다.
                deadline = now+timeout_s
            if results:
                result = results[0]
                if 'client_error' in result:
                    return 4, dict(outcome='unknown_after_send', command=command, error=result['client_error'], boundary=BOUNDARY)
                return (0 if result['accepted'] else 3), dict(
                    outcome='accepted_policy_only' if result['accepted'] else 'rejected',
                    command=command, result=result, boundary=BOUNDARY)
        counts = dict(command_readers=len(node.get_subscriptions_info_by_topic(COMMAND)),
            result_writers=len(node.get_publishers_info_by_topic(RESULT)),
            status_writers=len(node.get_publishers_info_by_topic(STATUS)))
        return (4 if sent else 2), dict(outcome='unknown_after_send' if sent else 'not_sent',
            error='timeout; do not automatically retry', endpoints=counts, command=command, boundary=BOUNDARY)
    except (Exception, KeyboardInterrupt) as error:
        return (4 if sent else 2), dict(outcome='unknown_after_send' if sent else 'not_sent',
            error=type(error).__name__+': '+str(error), command=command, boundary=BOUNDARY)
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('status', 'stop', 'new-race', 'acknowledge'))
    parser.add_argument('--session')
    parser.add_argument('--race-epoch', type=int)
    parser.add_argument('--intervention', type=int)
    parser.add_argument('--hazards-cleared', action='store_true')
    parser.add_argument('--operator-reviewed', action='store_true')
    parser.add_argument('--send', action='store_true', help='조회/명령 표시만이 아니라 상위 정책 명령 1회 전송')
    parser.add_argument('--timeout-s', type=float, default=5.)
    args = parser.parse_args(argv)
    op = args.operation.replace('-', '_')
    try:
        if op == 'status':
            if args.send or any(x is not None for x in (args.session, args.race_epoch, args.intervention)) or args.hazards_cleared or args.operator_reviewed:
                raise ValueError('status accepts no mutation flags')
            code, report = exchange(op, timeout_s=args.timeout_s)
        else:
            command = build_command(op, session=args.session, race_epoch=args.race_epoch,
                intervention=args.intervention, hazards_cleared=args.hazards_cleared, operator_reviewed=args.operator_reviewed)
            if args.send:
                code, report = exchange(op, command, timeout_s=args.timeout_s)
            else:
                code, report = 0, dict(outcome='dry_run_no_ROS_connection', command=command, boundary=BOUNDARY)
    except (ValueError, ImportError) as error:
        code, report = 2, dict(outcome='not_sent', error=str(error), boundary=BOUNDARY)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
