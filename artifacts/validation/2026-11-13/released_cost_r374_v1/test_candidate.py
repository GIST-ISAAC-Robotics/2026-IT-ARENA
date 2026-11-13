"""호출 간 오염·반환 별칭·예외 복구 계약. 실제 기하 동등성은 compare.py가 검사한다."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(A), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import numpy as np
import candidate
from arena_autonomy import pass_release_trial as ps


class CacheContract(unittest.TestCase):
    def setUp(self):
        self.scene, self.cfg, self.lat = object(), object(), object()
        self.calls = 0

    def region(self, scene, speed, target, steering, previous, dt, cfg, lat, *, context=None):
        self.calls += 1
        if context is not None:
            context.update(queries=np.array([[.1, .2]]), padding=np.array([.03]),
                           free=np.array([False]), masked=np.array([True]))
        return dict(queries=1, unobserved=0, distance_m=max(abs(speed), abs(target)))

    def ask(self, *, target=.35, speed=.1, scene=None, steering=.1, cfg=None, context=None):
        return ps.motion_region(self.scene if scene is None else scene, speed, target, steering, .1, .1,
                                self.cfg if cfg is None else cfg, self.lat, context=context)

    def test_context_none_then_context_and_mutation_do_not_alias(self):
        def selected(owner):
            first = self.ask()
            first['distance_m'] = 999.
            second_context = {'caller': 7}
            second = self.ask(context=second_context)
            self.assertEqual(second['distance_m'], .35)
            second_context['queries'][0, 0] = 999.
            second_context.clear()
            final_context = {'caller': 9}
            final = self.ask(context=final_context)
            self.assertEqual(final_context['caller'], 9)
            self.assertEqual(final_context['queries'][0, 0], .1)
            return final
        with patch.object(ps, 'motion_region', self.region), patch.object(ps.PassReleaseSelector, 'select', selected):
            with candidate.enabled():
                ps.PassReleaseSelector.select(object())
        self.assertEqual(self.calls, 1)

    def test_each_select_scope_recomputes(self):
        with patch.object(ps, 'motion_region', self.region), patch.object(ps.PassReleaseSelector, 'select', lambda owner: self.ask()):
            with candidate.enabled():
                ps.PassReleaseSelector.select(object())
                ps.PassReleaseSelector.select(object())
        self.assertEqual(self.calls, 2)

    def test_scene_config_and_steering_changes_do_not_reuse(self):
        def selected(owner):
            self.ask()
            self.ask(scene=object())
            self.ask(cfg=object())
            self.ask(steering=.10000000000000002)
        with patch.object(ps, 'motion_region', self.region), patch.object(ps.PassReleaseSelector, 'select', selected):
            with candidate.enabled():
                ps.PassReleaseSelector.select(object())
        self.assertEqual(self.calls, 4)

    def test_maximum_speed_shared_and_full_hit_verification(self):
        rows = []
        def selected(owner):
            self.ask(speed=.4, target=.35)
            self.ask(speed=.4, target=.25)
        with patch.object(ps, 'motion_region', self.region), patch.object(ps.PassReleaseSelector, 'select', selected):
            with candidate.enabled(observer=rows.append, verify_hits=True):
                ps.PassReleaseSelector.select(object())
        self.assertEqual(self.calls, 2)  # 적중 시 검증용 원 계산 1회 포함
        self.assertEqual(rows[0]['hits'], 1)
        self.assertEqual(rows[0]['verified_hits'], 1)

    def test_exception_restores_functions_and_scope(self):
        original_region = ps.motion_region
        original_select = ps.PassReleaseSelector.select
        with self.assertRaisesRegex(RuntimeError, 'deliberate'):
            with candidate.enabled():
                raise RuntimeError('deliberate')
        self.assertIs(ps.motion_region, original_region)
        self.assertIs(ps.PassReleaseSelector.select, original_select)
        self.assertIsNone(candidate._SCOPE.get())


if __name__ == '__main__':
    unittest.main()
