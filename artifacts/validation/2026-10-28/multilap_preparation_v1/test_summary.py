"""랩별 보조 요약이 시작 전/종료 뒤 진행을 완주에 더하지 않는지 검사한다."""
from summarize import lap_windows, sampled_side_entries


def row(t, progress):
    return dict(t=t, progress_m=progress, ground_speed_mps=.35, safe_speed_mps=.35,
                autonomy=dict(reason='following'))


def test_absolute_offset_and_post_window_crossing_do_not_add_laps():
    rows = [row(0, 99), row(1, 100), row(2, 109), row(3, 110), row(4, 119), row(5, 120)]
    result = lap_windows(rows, 1, 4, 10, 2)
    assert result['sampled_origin_m'] == 100
    assert result['sampled_origin_time_s'] == 1
    assert len(result['windows']) == 1
    assert result['windows'][0]['reached_s'] == 3


def test_start_line_backtracking_is_not_a_new_net_lap():
    rows = [row(t, p) for t, p in enumerate([0, 9, 11, 9, 11, 9, 11])]
    result = lap_windows(rows, 0, 6, 10, 5)
    assert len(result['windows']) == 1


def test_monotonic_five_laps_and_original_samples_remain_unchanged():
    rows = [row(t, 100 + 10*t) for t in range(6)]
    before = repr(rows)
    result = lap_windows(rows, 0, 5, 10, 5)
    assert [r['reached_s'] for r in result['windows']] == [1, 2, 3, 4, 5]
    assert [r['duration_s'] for r in result['windows']] == [1]*5
    assert repr(rows) == before


def test_empty_evaluation_window_has_no_claimed_origin_or_laps():
    result = lap_windows([row(0, 100)], 1, 2, 10, 5)
    assert result == dict(sampled_origin_m=None, sampled_origin_time_s=None, windows=[])


def side_row(t, present, passed=False, clear=0):
    result = row(t, t)
    result['pass_release_trial'] = dict(selector_mode='GUARD', selector_sides={
        'left': dict(present=present, passed=passed, clear=clear)})
    return result


def test_second_visible_encounter_is_counted_and_previous_pass_is_not_reused():
    rows = [side_row(0, False), side_row(.1, True), side_row(.2, True),
            side_row(.3, False, True, 2), side_row(.4, True)]
    entries = sampled_side_entries(rows, 0, .4)
    assert [item['t'] for item in entries] == [.1, .4]
    assert all(item['preceded_by_observed_absence'] and item['previous_pass_not_reused'] for item in entries)


def test_missing_status_and_time_gap_do_not_claim_observed_transition():
    rows = [side_row(0, False), row(.1, .1), side_row(.2, True), side_row(1, True)]
    entries = sampled_side_entries(rows, 0, 1)
    assert len(entries) == 2
    assert all(not item['preceded_by_observed_absence'] for item in entries)


def test_inconsistent_passed_state_is_reported_instead_of_repaired():
    entries = sampled_side_entries([side_row(0, True, True, 2)], 0, 1)
    assert len(entries) == 1 and not entries[0]['previous_pass_not_reused']
    assert entries[0]['passed'] is True and entries[0]['clear'] == 2
