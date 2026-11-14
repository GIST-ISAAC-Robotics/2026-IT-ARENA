#!/usr/bin/env python3
"""Per-SSH-session servo controls. Commands arrive on stdin; no network listener."""
import argparse
import json
import math
from pathlib import Path
import sys
import tempfile

from mounted_servo_setup import (ERROR, SPAN, STEP, Probe, StepNotReached, bounds, checked_feedback, jog, read_bytes,
                                 position, record_zero, release, require_torque_off, save_new_profile,
                                 summarize_manual)

END_SEARCH_SPAN = 275  # CAD search ceiling, not a measured clearance limit.
CLICK_SIZES = (STEP, 2*STEP, 3*STEP, 4*STEP)


class Session:
    def __init__(self, args):
        self.folder = Path(tempfile.mkdtemp(prefix='panel-run-', dir=args.runs))
        self.probe = Probe(args.port, self.folder / 'relay')
        self.points = {}
        try:
            info = self.probe.connect()
            if info['armed'] or (info['tx_gpio'], info['rx_gpio']) != (17, 18):
                raise RuntimeError('diagnostic relay must be in standby')
            self.probe.command('B 1000000', 'baud')
            self.probe.servo_id = 1
        except BaseException:
            self.probe.close()
            raise
        self.origin = None
        self.config = None

    def sample(self):
        p = self.probe
        require_torque_off(p)
        config = read_bytes(p, 0, 40)
        low, high = bounds(config)
        if self.config is not None and self.config != config:
            raise RuntimeError('servo settings changed; start a new session')
        current = position(checked_feedback(p, .25, 'panel_sample', low, high), torque_off=True)
        require_torque_off(p)
        self.config = config
        if self.origin is None:
            self.origin = current
        return current, low, high

    def execute(self, request):
        p = self.probe
        action = request.get('action')
        if action == 'release':
            return {'torque_after': release(p)}
        if action not in ('read', 'step', 'capture'):
            raise ValueError('unknown panel request')
        current, low, high = self.sample()
        result = dict(position=current, torque_after=0, points=self.points, folder=str(self.folder))
        if action == 'step':
            delta = request.get('delta')
            if type(delta) is not int or abs(delta) not in CLICK_SIZES:
                raise ValueError('한 번에 12, 24, 36, 48 counts 중 하나만 이동할 수 있습니다.')
            target = current + delta
            straight = self.points.get('straight')
            origin = straight['shaft_count'] if straight else self.origin
            span = END_SEARCH_SPAN if straight and request.get('find_ends') is True else SPAN
            if not low+ERROR <= min(current, target) <= max(current, target) <= high-ERROR:
                raise ValueError('서보 수치 끝에 가까워 이동을 막았습니다.')
            if abs(target-origin) > span and abs(target-origin) >= abs(current-origin):
                raise ValueError('탐색 범위 끝입니다. 중앙을 저장하거나 중앙 쪽으로 움직이세요.')
            for key in ('left_max', 'right_max'):
                if key in self.points:
                    end = self.points[key]['shaft_count']
                    if (end-origin)*(target-end) > 0 and (end-origin)*delta > 0:
                        raise ValueError('저장한 끝점 바깥으로는 이동할 수 없습니다.')
            # The existing firmware independently checks +/-114 from fresh ARM position.
            result.update(start=current, target=target, torque_after=None)
            try:
                jog(p, self.config, current, target, low, high, result)
            except StepNotReached:
                if result['torque_after'] != 0:
                    raise
                result['step_reached'] = False
            else:
                result['step_reached'] = True
            after = checked_feedback(p, .15, 'panel_released', low, high)
            result.update(position=after[-1]['position_counts'])
        elif action == 'capture':
            stages = ('straight', 'left_max', 'right_max')
            expected = next((key for key in stages if key not in self.points), None)
            if not expected or request.get('stage') != expected:
                raise ValueError('중앙 → 왼쪽 최대 → 오른쪽 최대 순으로 저장하세요.')
            point = dict(shaft_count=current, wheel_angle_deg=None)
            if expected == 'straight':
                point.update(shaft_delta_counts_from_straight=0, shaft_delta_deg_from_straight=0.0)
                profile = record_zero(current, assembly_id='arena-car-v1', straight_confirmed=True,
                                      evidence_note='Operator visually confirmed straight in steering panel')
                save_new_profile(self.folder / 'straight.json', profile)
            else:
                zero = self.points['straight']['shaft_count']
                if not ERROR < abs(current - zero) <= END_SEARCH_SPAN:
                    raise ValueError('중앙과 구분되는 위치를 찾은 뒤 저장하세요.')
                if expected == 'right_max' and (current-zero)*(self.points['left_max']['shaft_count']-zero) >= 0:
                    raise ValueError('오른쪽 끝은 중앙을 기준으로 왼쪽 끝과 반대편이어야 합니다.')
                angles = request.get('angles')
                if angles is not None:
                    if (not isinstance(angles, list) or len(angles) != 2 or
                            any(type(a) not in (float, int) or not math.isfinite(a) or not 0 < a <= 90 for a in angles)):
                        raise ValueError('바퀴 실측각은 0° 초과, 90° 이하 두 값입니다.')
                    sign = 1 if expected == 'left_max' else -1
                    point['wheel_angle_deg'] = dict(left_wheel=sign*angles[0], right_wheel=sign*angles[1])
            self.points[expected] = point
            if expected == 'right_max':
                result.update(summarize_manual(self.points, low, high))
            summary = dict(result, assembly_id='arena-car-v1', operating_limits_confirmed=False,
                           completed=expected == 'right_max', manual_points=self.points)
            tmp = self.folder / 'summary.tmp'
            tmp.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
            tmp.replace(self.folder / 'summary.json')
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--runs', required=True, type=Path)
    args = parser.parse_args()
    session = None
    print('{"ready": true}', flush=True)
    try:
        for line in sys.stdin:
            try:
                request = json.loads(line)
                if request.get('action') == 'close':
                    break
                if session is None:
                    session = Session(args)
                response = {'ok': True, **session.execute(request)}
            except Exception as exc:
                response = {'ok': False, 'error': str(exc), 'check_again': not isinstance(exc, ValueError)}
            print(json.dumps(response, ensure_ascii=True), flush=True)
    finally:
        if session is not None:
            session.probe.close()


if __name__ == '__main__':
    main()
