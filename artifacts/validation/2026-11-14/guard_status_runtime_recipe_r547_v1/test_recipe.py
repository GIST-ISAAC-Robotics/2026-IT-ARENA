"""Windows 파일 fixture/AST만 검사. ROS·장치·WSL 실행 없음."""
import ast
import copy
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path, PurePosixPath
import sys
import tempfile
import unittest

A = Path(__file__).resolve().parent
R536 = A.parent/'current_delivery_audit_r536_v1/stage'
R543 = A.parent/'guard_status_payload_r543_v1/stage_integration'
sys.path.insert(0, str(A/'stage'))
from prepare import overlay, prepare, sha, SIDECAR
from collect import collect


class RecipeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=A)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.profile = json.loads((R536/'profile.compact.json').read_bytes())
        # 현재 선택자가 준 보정/포트 값은 어떤 값이든 이 도구가 건드리지 않아야 한다.
        self.profile['lidar_safety']['ros__parameters']['fixture_selected_limit'] = .28475436568260193
        self.profile['extra'] = {'port':6241, 'frame':'selected-frame', 'calibration':{'neutral':2811}}
        self.base = self.root/'base.json'
        self.base.write_text(json.dumps(self.profile), encoding='utf-8')

    def package(self, mode='compact_v1'):
        return prepare(self.base, self.root/'plans', '/future/r547-results', mode)

    def capture(self, package, values, detail=None):
        source = self.root/('result_'+package.name)
        source.mkdir()
        (source/'invocation.json').write_text(json.dumps({'profile_sha256':sha(package/'profile.json')}))
        (source/'commands.jsonl.gz').write_bytes(b'unchanged command capture fixture')
        (source/'execution.json').write_text(json.dumps({'residual_groups':[], 'forced':[]}))
        rows = [dict(received_s=12.345+i*.02, received_wall_s=500.25+i*.02, value=v)
                for i,v in enumerate(values)]
        with gzip.open(source/'guard_status.jsonl.gz', 'wt', encoding='utf-8') as stream:
            for row in rows:stream.write(json.dumps(row)+'\n')
        if detail is not None:(source/SIDECAR).write_text(detail, encoding='utf-8')
        return source, rows

    def detail(self, archive='archive-A'):
        context = {'source_first_s':12.1, 'geometry':[[1., 2.]], 'epoch':7}
        encoded = json.dumps(context).encode()
        ref = dict(archive_id=archive, record_id=1, sha256=hashlib.sha256(encoded).hexdigest(), bytes=len(encoded))
        record = dict(schema='guard_status_context_v1', **ref, data=context)
        full = dict(reason='command_stale', source_s=12.125,
                    partial_observation={'latch_evidence':{'dynamic':.25, 'correspondence_context':context}})
        compact = copy.deepcopy(full)
        evidence = compact['partial_observation']['latch_evidence']
        del evidence['correspondence_context']
        evidence['correspondence_context_ref'] = ref
        compact['status_detail'] = dict(schema='guard_status_compact_v1', mode='compact_v1',
            context_storage='written', full_context_in_message=False,
            archive={'path':SIDECAR, 'archive_id':archive})
        return full, compact, record

    def test_full_default_and_only_two_guard_fields_change(self):
        original = copy.deepcopy(self.profile)
        package = self.package('full')
        selected = json.loads((package/'profile.json').read_bytes())
        guard = selected['lidar_safety']['ros__parameters']
        self.assertEqual(guard.pop('safety_status_detail'), 'full')
        self.assertEqual(guard.pop('safety_status_evidence_path'), '')
        self.assertEqual(selected, original)
        self.assertEqual(self.profile, original)
        self.assertIsNone(json.loads((package/'selection.json').read_bytes())['target_sidecar'])

    def test_compact_new_unique_run_and_only_two_guard_fields(self):
        packages = [self.package(), self.package()]
        plans = [json.loads((p/'selection.json').read_bytes()) for p in packages]
        self.assertNotEqual(plans[0]['target_result'], plans[1]['target_result'])
        for package, plan in zip(packages, plans):
            self.assertTrue(PurePosixPath(plan['target_sidecar']).is_absolute())
            self.assertEqual(PurePosixPath(plan['target_sidecar']).parent, PurePosixPath(plan['target_result']))
            selected = json.loads((package/'profile.json').read_bytes())
            guard = selected['lidar_safety']['ros__parameters']
            self.assertEqual(guard.pop('safety_status_detail'), 'compact_v1')
            self.assertEqual(guard.pop('safety_status_evidence_path'), plan['target_sidecar'])
            self.assertEqual(selected, self.profile)

    def test_invalid_path_or_implicit_planner_compact_rejected(self):
        for target in ('relative/path', '/', '/future/../other'):
            with self.subTest(target=target), self.assertRaises(ValueError):
                overlay(self.profile, 'compact_v1', target)
        self.profile['local_pursuit']['ros__parameters']['g1_trace_detail'] = 'full'
        with self.assertRaises(ValueError):overlay(self.profile, 'compact_v1', '/future/new')

    def test_existing_launcher_preserves_guard_overlay(self):
        spec = importlib.util.spec_from_file_location('r536_contracts', R536/'contracts.py')
        contracts = importlib.util.module_from_spec(spec);spec.loader.exec_module(contracts)
        selected = overlay(self.profile, 'compact_v1', '/future/new')
        prepared = contracts.prepared_profile(selected, self.root/'out')
        self.assertEqual(selected['lidar_safety'], prepared['lidar_safety'])
        self.assertEqual(prepared['local_pursuit']['ros__parameters']['g1_trace_detail'], 'compact_v1')

    def test_compact_full_pending_restore_all_fields_and_raw_pair(self):
        package = self.package()
        full, compact, record = self.detail()
        pending = copy.deepcopy(full)
        pending['status_detail'] = {**compact['status_detail'], 'context_storage':'pending', 'full_context_in_message':True}
        source, rows = self.capture(package, [pending, compact, full], json.dumps(record)+'\n')
        before = {p.name:sha(p) for p in source.iterdir()}
        destination = self.root/'collected'
        result = collect(package/'selection.json', source, destination)
        self.assertTrue(result['restored'] and result['compact_reduction_observed'])
        self.assertTrue(result['compact_with_full_rows'])
        self.assertEqual(result['mode_counts'], {'compact_v1':2, 'full':1})
        with gzip.open(destination/'guard_status.restored.jsonl.gz', 'rt', encoding='utf-8') as stream:
            restored = [json.loads(line) for line in stream]
        self.assertEqual(restored, [{**row, 'value':full} for row in rows])
        self.assertEqual(before, {p.name:sha(p) for p in source.iterdir()})
        self.assertEqual(before, {p.name:sha(p) for p in (destination/'runtime').iterdir()})

    def test_full_round_trip_without_sidecar(self):
        package = self.package('full')
        full, _, _ = self.detail()
        source, rows = self.capture(package, [full])
        result = collect(package/'selection.json', source, self.root/'full')
        self.assertTrue(result['restored'])
        self.assertFalse(result['sidecar_present'] or result['compact_reduction_observed'])

    def test_compact_requested_but_full_is_not_marked_observed(self):
        package = self.package()
        full, _, _ = self.detail()
        source, _ = self.capture(package, [full])
        result = collect(package/'selection.json', source, self.root/'fallback')
        self.assertTrue(result['requested_compact_but_unobserved'])
        self.assertFalse(result['compact_reduction_observed'])

    def test_missing_tampered_or_other_run_detail_rejected_raw_preserved(self):
        for failure in ('missing', 'tampered', 'other_run'):
            with self.subTest(failure=failure):
                package = self.package()
                _, compact, record = self.detail()
                if failure == 'tampered':record['data']['epoch'] += 1
                if failure == 'other_run':record['archive_id'] = 'another-run'
                detail = None if failure == 'missing' else json.dumps(record)+'\n'
                source, _ = self.capture(package, [compact], detail)
                destination = self.root/failure
                with self.assertRaises(ValueError):collect(package/'selection.json', source, destination)
                receipt = json.loads((destination/'collection.json').read_bytes())
                self.assertFalse(receipt['restored'])
                self.assertTrue(receipt['error'])
                self.assertEqual(sha(source/'guard_status.jsonl.gz'), sha(destination/'runtime/guard_status.jsonl.gz'))

    def test_recollection_never_overwrites(self):
        package = self.package('full')
        source, _ = self.capture(package, [{'reason':'ok'}])
        destination = self.root/'fresh'
        collect(package/'selection.json', source, destination)
        before = sha(destination/'collection.json')
        with self.assertRaises(FileExistsError):collect(package/'selection.json', source, destination)
        self.assertEqual(before, sha(destination/'collection.json'))

    def test_wrong_profile_receipt_rejected(self):
        package = self.package('full')
        source, _ = self.capture(package, [{'reason':'ok'}])
        (source/'invocation.json').write_text(json.dumps({'profile_sha256':'wrong'}))
        with self.assertRaises(ValueError):collect(package/'selection.json', source, self.root/'wrong')

    def test_restoration_function_asts_identical_to_r543(self):
        def functions(path):
            return {n.name:ast.dump(n, include_attributes=False) for n in ast.parse(path.read_text(encoding='utf-8')).body
                    if isinstance(n, ast.FunctionDef) and n.name in ('load_details','restore_status')}
        self.assertEqual(functions(R543/'guard_status_payload.py'), functions(A/'stage/restore_context.py'))

    def test_r536_unchanged_and_no_added_control_or_drain(self):
        manifest = json.loads((R536/'stage_manifest.json').read_bytes())
        for name, digest in manifest['files'].items():self.assertEqual(sha(R536/name), digest)
        tree = ast.parse((R536/'observation.py').read_text(encoding='utf-8'))
        control = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ControlReceipt')
        calls = [n for n in ast.walk(control) if isinstance(n, ast.Call)]
        self.assertEqual(sum(isinstance(c.func,ast.Attribute) and c.func.attr=='control' for c in calls), 1)
        for path in (A/'stage').glob('*.py'):
            tree = ast.parse(path.read_text(encoding='utf-8'))
            self.assertFalse(any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                                 and n.func.attr in ('control','drain_pending','publish') for n in ast.walk(tree)))


if __name__ == '__main__':
    unittest.main(verbosity=2)
