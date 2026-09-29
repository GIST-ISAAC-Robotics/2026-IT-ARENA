import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_imu_delivery import analyze_rows, loss_runs, publication_evidence, compare_probe


def fixture():
    expected = [i*4_000_000 for i in range(1, 9)]
    rows = []
    for name, kept in [('local_pursuit', [1, 4, 5, 6, 8]), ('lidar_safety', [1, 2, 3, 4, 5, 6, 8])]:
        for i in kept:
            rows.append(dict(node=name, callback='imu', source_ns=i*4_000_000,
                begin_monotonic_ns=i*4_000_000+1_000_000, source_age_ms=1., duration_ms=.1))
    report = dict(active_window_continuity=dict(reference_start_ns=12_000_000, reference_end_ns=24_000_000),
        diagnostic_sequence_coverage={n+'/imu':dict(complete=True) for n in ['local_pursuit','lidar_safety']},
        input_callback_delivery={n+'/imu':dict(callbacks=c,published=8) for n,c in [('local_pursuit',5),('lidar_safety',7)]})
    return expected, rows, report


class ImuDeliveryTests(unittest.TestCase):
    def test_probe_can_receive_controller_missing_samples(self):
        source, rows, _ = fixture()
        probe = [dict(source_ns=s, begin_monotonic_ns=s+1000) for s in source]
        result = compare_probe(source, rows, probe)
        self.assertEqual(result['missing'], 0)
        self.assertEqual(result['comparisons']['local_pursuit']['controller_missing_probe_received'], 3)

    def test_probe_independent_and_common_loss(self):
        source, rows, _ = fixture()
        probe = [dict(source_ns=s, begin_monotonic_ns=s+1000) for s in source if s not in (4_000_000,28_000_000)]
        result = compare_probe(source, rows, probe)
        self.assertEqual(result['missing'], 2)
        self.assertEqual(result['comparisons']['local_pursuit'],
            dict(common_missing=1,controller_missing_probe_received=2,probe_missing_controller_received=1))

    def test_probe_invalid_timestamps_rejected(self):
        source, rows, _ = fixture()
        for stamps in ([4_000_000,4_000_000], [999]):
            with self.assertRaises(ValueError):
                compare_probe(source, rows, [dict(source_ns=s,begin_monotonic_ns=s) for s in stamps])

    def test_loss_runs_and_brackets(self):
        runs = loss_runs([4,8,12,16,20,24], {4,12,16,24})
        self.assertEqual([r['count'] for r in runs], [1,2,1])
        self.assertIsNone(runs[0]['before_source_ns'])
        self.assertIsNone(runs[-1]['after_source_ns'])
        self.assertEqual(runs[1]['bracketing_source_gap_ms'], 12/1e6)

    def test_loss_and_shared_observations(self):
        out = analyze_rows(*fixture())
        local = out['local_pursuit']
        self.assertEqual(local['missing'], 3)
        self.assertEqual(local['longest_missing_run'], 2)
        self.assertEqual([local[k] for k in ['missing_before_active','missing_in_active','missing_after_active']], [1,1,1])
        self.assertEqual(local['source_gap_ms']['maximum'], 12.)
        self.assertEqual(out['comparison'], dict(common_missing=1,local_missing_but_safety_received=2,safety_missing_but_local_received=0))

    def test_diagnostic_or_source_count_mismatch_rejected(self):
        for change in ['coverage','callbacks','published']:
            source, rows, report = fixture()
            if change == 'coverage': report['diagnostic_sequence_coverage']['local_pursuit/imu']['complete'] = False
            else: report['input_callback_delivery']['local_pursuit/imu'][change] += 1
            with self.assertRaises(ValueError): analyze_rows(source, rows, report)

    def test_ambiguous_source_rejected(self):
        source, rows, report = fixture()
        with self.assertRaises(ValueError): analyze_rows(source+[source[-1]], rows, report)
        with self.assertRaises(ValueError): analyze_rows(source[::-1], rows, report)

    def test_duplicate_or_unexpected_received_rejected(self):
        for stamp in [4_000_000,999]:
            source, rows, report = fixture()
            rows[1]['source_ns'] = stamp
            with self.assertRaises(ValueError): analyze_rows(source, rows, report)

    def test_source_reversal_reported_not_sorted_away(self):
        source, rows, report = fixture()
        rows[0]['source_ns'], rows[1]['source_ns'] = rows[1]['source_ns'], rows[0]['source_ns']
        self.assertEqual(analyze_rows(source, rows, report)['local_pursuit']['source_time_reversals'], 1)

    def test_publication_burst(self):
        source, rows, _ = fixture()
        pubs = [dict(source_ns=s,begin_monotonic_ns=i*100_000,end_monotonic_ns=i*100_000+10_000,
                     schedule_lateness_wall_ms=2.) for i,s in enumerate(source)]
        out = publication_evidence(pubs,rows,source)
        self.assertEqual(out['max_submillisecond_cluster'],8)
        self.assertEqual(out['clusters_at_least_6'],1)
        self.assertEqual(out['nodes']['local_pursuit']['missing_in_clusters_at_least_6'],3)

    def test_publication_negative_end_interval_preserved(self):
        pub = dict(source_ns=4_000_000,begin_monotonic_ns=0,end_monotonic_ns=10_000,
                   schedule_lateness_wall_ms=0.)
        rows = [dict(node=n,callback='imu',source_ns=4_000_000,begin_monotonic_ns=5_000)
                for n in ['local_pursuit','lidar_safety']]
        out = publication_evidence([pub],rows,[4_000_000])
        self.assertEqual(out['nodes']['local_pursuit']['publish_end_to_callback_ms']['maximum'], -.005)

    def test_publications_between_callback_boundaries(self):
        pubs = [dict(source_ns=i,begin_monotonic_ns=i*100_000,end_monotonic_ns=i*100_000+10_000,
                     schedule_lateness_wall_ms=0.) for i in range(1,9)]
        rows = [dict(node=n,callback='imu',source_ns=s,begin_monotonic_ns=t)
                for n in ['local_pursuit','lidar_safety'] for s,t in [(1,120_000),(8,900_000)]]
        out = publication_evidence(pubs,rows,list(range(1,9)))['nodes']['local_pursuit']
        self.assertEqual(out['windows_more_than_5_publications_between_callbacks'][0]['publications'],7)
        self.assertEqual(out['missing_published_in_those_windows'],6)

    def test_publication_missing_and_backwards_rejected(self):
        source, rows, _ = fixture()
        pubs = [dict(source_ns=s,begin_monotonic_ns=i*100_000,end_monotonic_ns=i*100_000+10_000,
                     schedule_lateness_wall_ms=2.) for i,s in enumerate(source)]
        with self.assertRaises(ValueError): publication_evidence(pubs[:-1], rows, source)
        pubs[1]['begin_monotonic_ns'] = -1
        with self.assertRaises(ValueError): publication_evidence(pubs, rows, source)


if __name__ == '__main__':
    unittest.main()
