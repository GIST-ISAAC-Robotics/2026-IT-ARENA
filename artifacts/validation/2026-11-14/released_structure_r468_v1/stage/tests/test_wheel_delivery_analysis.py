"""지연 분석의 시각 종류·중복·미계측 구분. ROS나 네트워크 없이 실행한다."""
import importlib.util
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('wheel_analysis',
    Path(__file__).resolve().parents[1]/'scripts/analyze_wheel_delivery.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class WheelAnalysisTest(unittest.TestCase):
    def analyze(self, trace=None, rows=None):
        root = Path('mock_replay')
        rows = rows if rows is not None else [dict(node='controller', callback='wheels', source_ns=10,
            source_age_ms=40., duration_ms=.1, begin_monotonic_ns=20_000_000, end_monotonic_ns=20_100_000)]
        content = {'report.json': '{}', 'timing.jsonl': ''.join(json.dumps(r)+'\n' for r in rows)}
        if trace is not None:
            content['wheel_publication.jsonl'] = ''.join(json.dumps(r)+'\n' for r in trace)
        with patch.object(Path, 'read_text', lambda path, **_: content[path.name]), \
             patch.object(Path, 'open', lambda path, **_: io.StringIO(content[path.name])), \
             patch.object(Path, 'exists', lambda path: path.name in content):
            return module.analyze(root)

    def test_absent_trace_is_not_zero_delay(self):
        result = self.analyze()
        self.assertFalse(result['wheel_publication_trace'])
        stats = result['nodes']['controller']
        self.assertEqual(stats['absent_publications'], 1)
        self.assertIsNone(stats['transport_and_dispatch_ms']['publish_begin_to_callback_ms']['maximum'])
        self.assertEqual(stats['over_ms'], {'30': 1, '100': 0, '500': 0})

    def test_same_host_intervals_and_negative_end_delay_preserved(self):
        result = self.analyze([dict(source_ns=10, begin_monotonic_ns=10_000_000,
            end_monotonic_ns=21_000_000, schedule_lateness_wall_ms=3.)])
        stats = result['nodes']['controller']['transport_and_dispatch_ms']
        self.assertEqual(stats['publish_begin_to_callback_ms']['maximum'], 10.)
        self.assertEqual(stats['publish_end_to_callback_ms']['maximum'], -1.)
        self.assertEqual(stats['publish_duration_ms']['maximum'], 11.)

    def test_duplicate_source_is_ambiguous_not_arbitrarily_matched(self):
        stats = self.analyze([{'source_ns': 10}, {'source_ns': 10}])['nodes']['controller']
        self.assertEqual(stats['ambiguous_publications'], 1)
        self.assertEqual(stats['matched_publications'], 0)

    def test_empty_data_has_no_fabricated_nodes(self):
        self.assertEqual(self.analyze(rows=[])['nodes'], {})

    def test_wait_window_excludes_arriving_callback_and_other_nodes(self):
        rows = [dict(node=n, callback=k, begin_monotonic_ns=t, duration_ms=.1)
                for n, k, t in [('a', 'imu', 10_000_000), ('b', 'imu', 15_000_000),
                                ('a', 'control', 30_000_000), ('a', 'wheels', 50_000_000)]]
        result = module.callbacks_during_wait(rows, 'a', 10_000_000, 50_000_000)
        self.assertEqual(set(result), {'imu', 'control'})
        self.assertEqual(result['imu']['count'], 1)
        self.assertEqual(result['control']['maximum_start_gap_in_window_ms'], 20.)

    def test_distribution_nearest_rank_and_empty(self):
        self.assertIsNone(module.distribution([])['p99'])
        self.assertEqual(module.distribution(list(range(100)))['p99'], 98)


if __name__ == '__main__':
    unittest.main()
