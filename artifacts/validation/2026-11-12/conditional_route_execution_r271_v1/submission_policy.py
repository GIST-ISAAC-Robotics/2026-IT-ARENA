"""보조 계산을 시작할 가치만 판단한다. 명령 허가/보호 판정은 하지 않는다."""
import math


def assess(*, now_s, now_wall_s, plan_submitted_wall_s, plan_budget_s,
           current_budget_s, last_application_s, last_command_reference_s,
           current_reference_s, source_first_s, path_origin_s):
    values = (now_s, now_wall_s, plan_submitted_wall_s, plan_budget_s,
              current_budget_s, last_application_s, last_command_reference_s,
              current_reference_s, source_first_s, path_origin_s)
    if not all(math.isfinite(x) for x in values):
        return dict(offer=False, reason='cost_or_command_not_ready')
    if (plan_budget_s <= 0 or current_budget_s <= 0 or now_wall_s < plan_submitted_wall_s
            or now_s < current_reference_s or current_reference_s < source_first_s
            or now_s < last_application_s or now_s < path_origin_s):
        return dict(offer=False, reason='scheduling_clock_boundary')
    remaining = max(0., plan_budget_s - (now_wall_s - plan_submitted_wall_s))
    plan_finish = now_s + remaining
    apply_at = max(now_s + current_budget_s, last_application_s + .1)
    normal_finish = plan_finish + current_budget_s
    detail = dict(estimated_plan_finish_s=plan_finish, estimated_current_apply_s=apply_at,
                  estimated_normal_apply_s=normal_finish,
                  original_command_deadline_s=last_command_reference_s + .2)
    # 완료된 새 계획은 보조 검사를 폐기한다. 50Hz 부모 한 틱의 여유를 둔다.
    if apply_at + .02 >= plan_finish:
        return dict(offer=False, reason='new_plan_will_supersede', **detail)
    if normal_finish <= last_command_reference_s + .2 - .02:
        return dict(offer=False, reason='new_plan_before_command_deadline', **detail)
    # 이미 성립 불가능한 시간 예산의 계산은 시작하지 않는다. 실제 적용 때 다시 검사한다.
    if (apply_at - source_first_s >= .3 or apply_at - current_reference_s >= .2
            or apply_at - path_origin_s >= .5):
        return dict(offer=False, reason='cannot_apply_within_existing_budget', **detail)
    return dict(offer=True, reason='useful_before_new_plan', **detail)
