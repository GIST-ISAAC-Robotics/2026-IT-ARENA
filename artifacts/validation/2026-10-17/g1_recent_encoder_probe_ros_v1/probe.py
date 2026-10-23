"""정지/ARM 상태에서 오래된 합성 엔코더를 한 번 보내 시험 진단 회수만 확인.

ROS v5의 원인 재현이나 전체 43조건 재시험이 아니다. 실제 운영 수신 코드와
시험 자식용 진단을 사용하고, 제어 명령 직접 발행/보호 우회/실물 연결은 없다.
"""
import json
from pathlib import Path
import sys

import validate_pipeline_faults_ros as validation


def run_probe(self):
    self.report.update(scope='one_old_encoder_injection_while_stationary_ros_probe',
        original_ros_v5_cause_reproduced=False, complete_pipeline_regression=False,
        drive_commands_injected=False, scenario='stationary_encoder_rejection_probe')
    for role in ('host', 'mcu', 'sink', 'safety', 'controller'):
        self.children.start(role)
    baseline = self.monitor.sequence
    self.wait(lambda: self.fresh('mcu', baseline, lambda v: v['feedback_valid']) and
              self.fresh('autonomy', baseline, lambda v: not v['started'] and
                         v['vision']['stats']['accepted'] >= 3), 'probe startup')
    self.children.register_vision(self.data('autonomy')['vision']['worker_pid'])
    self.graph_audit()
    self.wait(lambda: self.stopped(0), 'fresh startup zero outputs')
    self.stop_case('probe_boot_disarmed', lambda: None,
                   {'mcu': lambda v: v['state'] == 'DISARMED'}, no_resume=True)
    self.stop_case('probe_armed_stationary', lambda: self.call('arm'),
                   {'mcu': lambda v: v['state'] == 'ARMED',
                    'autonomy': lambda v: not v['started'] and v['speed_command_mps'] == 0.},
                   no_resume=True)

    def inject_old_sample():
        message = self.stamp(self.types['motor'](), self.t - .075)
        message.name = ['drive_motor_shaft']
        message.position = [self.spec.motor_angle_from_counts(self.last_counts)]
        message.velocity = [0.]
        self.report['injected'] = dict(publish_sim_s=self.t,
            capture_s=self.t-.075, counts=self.last_counts,
            nominal_age_us=75_000, state_before=self.data('mcu')['state'])
        self.publishers['motor'].publish(message)

    self.stop_case('probe_old_encoder_fault_latched', inject_old_sample,
        {'mcu': lambda v: v['state'] == 'FAULT_LATCHED' and v['reason'] == 'invalid_encoder',
         'sink': lambda v: v['blocked']}, no_resume=True)
    self.stop_case('probe_fresh_encoder_does_not_auto_clear', lambda: None,
        {'mcu': lambda v: v['state'] == 'FAULT_LATCHED' and
                         v['reason'] == 'invalid_encoder' and v['feedback_valid']}, no_resume=True)
    mcu_path = self.output / next(p['log'] for p in self.report['processes'] if p['role'] == 'mcu')
    rows = [json.loads(line.split('=', 1)[1]) for line in mcu_path.read_text().splitlines()
            if line.startswith('MCU_ENCODER_REJECTION=')]
    self.report['rejection_probe_rows'] = rows
    assert len(rows) == 1, 'one actual rejected call must be logged'
    row = rows[0]
    expected = self.report['injected']
    assert abs(row['capture_us'] - round(expected['capture_s']*1e6)) <= 1
    assert row['counts'] == [expected['counts']]
    assert row['age_us'] >= 75_000
    assert row['feedback_timeout_us'] == 50_000
    assert row['state_before'] == 'ARMED'
    assert row['reason_after'] == 'invalid_encoder'
    assert row['scope'] == 'validator_child_only'
    self.final_stop()


# 고장 시험 클래스의 실행 목록만 이 평가 프로세스에서 한정한다.
# 원래 검증기 및 모든 운영 파일은 수정하지 않고 본 파일도 snapshot에 포함한다.
validation.PipelineValidator.run = run_probe
validation.FIXTURE_SOURCES.append(Path(__file__).resolve().relative_to(validation.ROOT).as_posix())

if __name__ == '__main__':
    sys.exit(validation.main())
