"""방지턱 시험의 명시적인 거리/랩 계약. 평가기 기본 랩 수에 의존하지 않는다."""
import math


def route_contract(scope, laps, lap_length_m):
    if scope not in ('crossing','course') or type(laps) is not int or not 1<=laps<=5:
        raise ValueError('invalid trial scope or laps')
    if not math.isfinite(lap_length_m) or lap_length_m<=6.2:
        raise ValueError('invalid official lap length')
    if scope=='crossing' and laps!=1:
        raise ValueError('crossing trial does not accept multiple laps')
    target=6.2 if scope=='crossing' else laps*lap_length_m+2.
    arguments=['--laps',str(laps),'--target-progress-m',repr(target)]
    return dict(version=2,scope=scope,requested_laps=laps,lap_length_m=lap_length_m,
                target_progress_m=target,full_lap_validation=scope=='course',arguments=arguments)


def validate_result(contract, report):
    """주행 성공과 별도로 준비한 거리/랩 수와 실제 평가 조건을 대조한다."""
    checks=dict(
        requested_laps=report.get('requested_laps')==contract['requested_laps'],
        full_lap_validation=report.get('full_lap_validation')==contract['full_lap_validation'])
    for key in ('lap_length_m','target_progress_m'):
        value=report.get(key)
        checks[key]=isinstance(value,(int,float)) and math.isfinite(value) and math.isclose(
            value,contract[key],rel_tol=0.,abs_tol=1e-9)
    return dict(passed=all(checks.values()),checks=checks,
                scope='requested route identity only; separate from driving outcome')
