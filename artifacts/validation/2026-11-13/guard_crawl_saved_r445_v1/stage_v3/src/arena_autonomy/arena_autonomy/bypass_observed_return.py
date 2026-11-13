"""우회 도중 막힌 목표선보다 현재 중앙 경로가 나으면 복귀를 다시 비교한다.

완전한 측방 이동/정렬은 복귀 경로를 만들기 위한 필수 물리 조건이 아니다.
기존 복귀 띠·실측 운동·점유/관측·조향 검사를 통과한 명령만 선택한다.
미래 경로의 간섭 순위는 자유 공간이나 전체 복귀 성공의 증명이 아니다.
"""
import math
import numpy as np

from arena_autonomy.bypass_entry_trial import _compare_entries, entry_path
from arena_autonomy.observation_support import source_valid


class ObservedEntryReturn:
    def __init__(self):
        self.source = None
        self.confirmations = 0

    def evaluate(self, guide, scene, speed, previous, dt, primary):
        if not (guide.config.piecewise_wall_trial and guide.rear_wall_witness_enabled
                and guide.rejoin_enabled and guide.stationary_outer_boundary):
            return primary
        source = scene.source_first_s
        if source == self.source:
            return primary  # 재사용한 스캔은 새 경로 확인으로 세지 않는다.
        consecutive = (self.source is not None and source is not None
                       and 0 < source-self.source < guide.config.memory_ttl_s)
        self.source = source
        if not consecutive:
            self.confirmations = 0
        reference = primary.get('reference', {})
        rows = primary.get('candidates', [])
        if (not source_valid(scene) or guide.phase != 'entry' or guide.path is None
                or guide.wall_confirmations < 2 or getattr(guide, 'reverse_wait', False)
                or not all(math.isfinite(v) for v in (speed, previous, dt)) or speed < 0
                or reference.get('reason') != 'current_outer_wall_rechecked_for_remembered_guide'
                or len(rows) != 1 or rows[0].get('reason') not in
                    ('observed_entry_command_candidate', 'known_occupied_command_hold')):
            self.confirmations = 0
            return primary
        current = rows[0]
        progress = current.get('preview_return_progress_m')
        path = np.asarray(current.get('path_points', []), dtype=float)
        if (path.ndim != 2 or path.shape[1] != 2 or len(path) < 2
                or not isinstance(progress, (int, float)) or not math.isfinite(progress)
                or progress+guide.entry.path_step_m >= path[-1, 0]-path[0, 0]):
            self.confirmations = 0
            return primary
        wall = reference['walls'][guide.side]
        fit = (wall['slope'], wall['intercept_m'], *wall['observed_x_m'])
        band = guide._rejoin_band(scene, fit, wall=wall)
        detail = dict(applied=False, band=band, confirmations=0,
                      reason='current_return_band_unavailable', free_space_assumed=False)
        if not band['supported']:
            self.confirmations = 0
            return dict(primary, observed_entry_return=detail)
        # 같은 스캔의 같은 현재 외벽 분류를 후보마다 재사용한다. 점군이나
        # 측정 속도를 변경하지 않으며 실제 직전 조향도 모든 비교에 포함한다.
        padding = guide.padding_provider(scene, guide.config, guide.side, wall)
        current_end = float(path[0, 0]+progress)
        gain = max(2*guide.config.measurement_padding_m, 2*guide.entry.path_step_m)
        alternatives, attempts = [], []
        for transition in guide.entry.transitions_m:
            try:
                proposal = entry_path(fit, 0., guide.config, guide.entry, transition)
            except ValueError:
                continue
            if proposal[-1, 0]-proposal[0, 0] < transition:
                # 복귀 단계로 바꾸려면 관측 구간 안에서 중앙 목표까지 도달한다.
                # 잘린 전이를 중앙까지 완성된 경로로 기억하지 않는다.
                continue
            result = _compare_entries(scene, scene.t_ref_s, speed, previous, dt,
                config=guide.config, entry=guide.entry,
                padding_provider=lambda *args: padding,
                guide=dict(reference=reference, path=proposal, offset_m=0., transition_m=transition,
                    outer_side=guide.side, stationary_outer_boundary=True))
            row = result['candidates'][0] if result['candidates'] else {}
            end = (float(proposal[0, 0]+row['preview_return_progress_m'])
                   if 'preview_return_progress_m' in row else None)
            better = result['selected'] is not None and end is not None and end >= current_end+gain
            attempts.append(dict(transition_m=transition, reason=row.get('reason'),
                                 preview_end_x_m=end, better=better))
            if better:
                alternatives.append((end, transition, proposal, result))
        detail.update(reason='no_better_current_return', current_preview_end_x_m=current_end,
                      required_gain_m=gain, attempts=attempts)
        if not alternatives:
            self.confirmations = 0
            return dict(primary, observed_entry_return=detail)
        self.confirmations += 1
        detail.update(confirmations=self.confirmations, reason='distinct_return_candidates_wait')
        if self.confirmations < 2:
            return dict(primary, observed_entry_return=detail)
        end, transition, proposal, result = max(alternatives, key=lambda v:(v[0], v[1]))
        guide.path = proposal
        guide.offset, guide.transition, guide.phase = 0., transition, 'rejoin'
        guide.rejoin_confirmations, guide.center_confirmations = 2, 0
        detail.update(applied=True, reason='better_current_return_selected',
                      selected_preview_end_x_m=end, original_offset_m=current['offset_m'])
        return dict(primary, **{k:result[k] for k in ('reason','candidates','selected')},
                    entry_phase='rejoin', rejoin_band=band, rejoin_confirmations=2,
                    center_confirmations=0, observed_entry_return=detail)
