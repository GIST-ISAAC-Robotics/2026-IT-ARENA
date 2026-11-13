"""선택형 기존 encoder 경계 전달. 합성 fixture만 사용하며 출력/장치 없음."""
import ast
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(HERE/'stage/src/arena_vehicle_interface'), str(ROOT/'src/arena_vehicle_interface')]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


subject = load('stage_profile', HERE/'stage/scripts/runtime_installed_profile.py')
base = load('base_profile', ROOT/'scripts/runtime_installed_profile.py')
from arena_vehicle_interface.steering_calibration import record_zero, complete_mount_profile

# 기존 합성 fixture만 재사용한다. pytest/native/장치 시험은 import하거나 실행하지 않는다.
tree = ast.parse((ROOT/'tests/test_runtime_installed_profile.py').read_text(encoding='utf8'))
fixture = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'synthetic_inputs')
exec(compile(ast.Module(body=[fixture], type_ignores=[]), '<existing synthetic_inputs>', 'exec'))


class FeedbackLimitContract(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=HERE)
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)
        self.board, self.zero, self.bp, self.zp = synthetic_inputs(self.path)

    def test_omitted_field_preserves_existing_generated_headers(self):
        self.assertEqual(subject.headers(self.board, self.zero, self.board['assembly_id']),
                         base.headers(self.board, self.zero, self.board['assembly_id']))
        header = (HERE/'stage/src/arena_vehicle_interface/firmware_runtime_drive/main/board_profile.hpp').read_text(encoding='utf8')
        self.assertIn('double max_wheel_mps=5.0;', header)
        self.assertIn('c.max_wheel_mps=max_wheel_mps;', header)

    def test_explicit_boundary_round_trip_preserves_input_and_selection_only(self):
        # 13/3.6은 계약 시험 입력이다. 실제 운영 threshold/물리 상한으로 채택하지 않는다.
        limit = 13 / 3.6
        self.board['max_wheel_mps'] = limit
        self.bp.write_text(json.dumps(self.board), encoding='utf8')
        raw = self.bp.read_bytes()
        folder = self.path/'prepared'
        result = subject.prepare(self.bp, self.zp, self.board['assembly_id'], folder)
        self.assertEqual(subject.verify(folder), result)
        self.assertEqual((folder/'runtime_board.json').read_bytes(), raw)
        self.assertIn(('p.max_wheel_mps='+repr(limit)+';').encode(), (folder/'runtime_installed_profile.hpp').read_bytes())
        for flag in ('output_enabled', 'firmware_uploaded', 'cpp_valid_checked'):
            self.assertIs(result[flag], False)

    def test_invalid_explicit_boundary_rejected_before_output(self):
        for index, value in enumerate((0., -1., True, None, '3.6', float('nan'), float('inf'), 1.9)):
            with self.subTest(value=value):
                board = copy.deepcopy(self.board)
                board['max_wheel_mps'] = value
                self.bp.write_text(json.dumps(board), encoding='utf8')
                out = self.path/('rejected'+str(index))
                with self.assertRaises(ValueError):
                    subject.prepare(self.bp, self.zp, board['assembly_id'], out)
                self.assertFalse(out.exists())

    def test_exact_schema_and_tamper_guard_remain(self):
        bad = {**self.board, 'feedback_speed_limit_typo':3.6}
        with self.assertRaises(ValueError): subject.headers(bad, self.zero, bad['assembly_id'])
        self.board['max_wheel_mps'] = 13/3.6
        self.bp.write_text(json.dumps(self.board), encoding='utf8')
        folder = self.path/'prepared'
        subject.prepare(self.bp, self.zp, self.board['assembly_id'], folder)
        changed = json.loads((folder/'runtime_board.json').read_text(encoding='utf8'))
        changed['max_wheel_mps'] = 5.
        (folder/'runtime_board.json').write_text(json.dumps(changed), encoding='utf8')
        with self.assertRaisesRegex(ValueError, 'changed'):
            subject.verify(folder)


if __name__ == '__main__': unittest.main()
