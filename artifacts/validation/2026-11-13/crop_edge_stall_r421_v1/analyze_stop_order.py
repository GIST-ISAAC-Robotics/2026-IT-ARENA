"""소유 helper STOP과 원 평가기 STOP의 저장 시각/세대를 분리한다."""
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
SOURCE = A.parent/'crop_edge_drive_r418_v1'


def main():
    paths = ['early_stop.json', 'run1/report.json', 'run1/stop_trace.json']
    values = {name: json.loads((SOURCE/name).read_bytes()) for name in paths}
    helper = values['early_stop.json']; report = values['run1/report.json']
    trace = values['run1/stop_trace.json']; timing = report['actuation']['stop_latch_timing']
    final = report['actuation']['final_mcu']
    result = dict(input_sha256={name: hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() for name in paths},
        helper=dict(calls=helper['calls'], exact_request_time_recorded=False,
            final_mcu=helper['final_mcu'], final_output=helper['final_output'],
            stop_confirmed=helper['stop_confirmed']),
        evaluator=dict(abort=report['abort'], first_latched=timing['first_latched'],
            request_sim_s=report['stop_request_sim_s'], request_clock_ns=report['stop_request_clock_ns'],
            time_limits=timing['limits'], final_mcu=final, final_output=report['actuation']['final_output'],
            stop_checks=report['stop_checks'], stop=report['stop'],
            shutdown_clean=report['shutdown_clean'], remaining_test_pids=report['remaining_test_pids']),
        checks=dict(same_boot=helper['final_mcu']['boot'] == final['boot'],
            helper_final_generation=helper['final_mcu']['generation'], evaluator_final_generation=final['generation'],
            helper_final_status_before_evaluator_clock=(helper['final_mcu']['now_us']*1000 < report['stop_request_clock_ns']),
            dense_stop_first_s=trace[0]['time'], dense_stop_last_s=trace[-1]['time'], dense_samples=len(trace)),
        interpretation='owned helper STOP is observed before the evaluator later issues its own STOP; '
            'first latch status is not exact request time; retain original pre-evaluator-STOP latch failure')
    with (A/'stop_order.json').open('x', encoding='utf8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2);stream.write('\n')
    print(json.dumps(result['checks']))


if __name__ == '__main__':
    main()
