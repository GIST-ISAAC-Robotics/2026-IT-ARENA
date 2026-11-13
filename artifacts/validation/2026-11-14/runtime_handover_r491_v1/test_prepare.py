"""파일 수집·누락 표시·비밀/빌드 산출물 제외만 검사. ROS/장치 실행 없음."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('candidate', HERE/'stage/scripts/prepare_runtime_handover.py')
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)


class BundleFiles(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=HERE)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def put(self, relative, raw=b'# fixture only\n'):
        p = self.root/relative
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(raw)

    def test_recursive_new_policy_and_build_sources(self):
        names = [candidate.PREFIX+'future/new_policy.py', candidate.PREFIX+'_native/current_mask_kernel.cpp',
                 'scripts/build_runtime_drive.sh', 'src/arena_vehicle_interface/firmware_runtime_drive/CMakeLists.txt']
        for n in names: self.put(n)
        self.assertEqual(set(candidate.collect(self.root)), set(names))

    def test_pending_r489_is_explicit(self):
        files = {n:b'# fixture\n' for n in candidate.REQUIRED if not n.endswith('independent_guard_route.py')}
        report = candidate.inventory(files)
        self.assertFalse(report['source_complete'])
        self.assertEqual(report['required_missing'], [candidate.PREFIX+'independent_guard_route.py'])
        self.assertTrue(all(value is None for value in report['installed_values'].values()))

    def test_no_cache_binary_secret(self):
        for n in ('scripts/private/password.py', 'scripts/.private/key.py', 'scripts/password.json',
                  candidate.PREFIX+'__pycache__/bad.py', candidate.PREFIX+'_numeric_kernels.so'):
            self.put(n)
        self.assertEqual(candidate.collect(self.root), {})

    def test_only_allowlisted_public_resources_no_installed_or_credential_json(self):
        for name in candidate.PUBLIC_RESOURCES:
            self.put(name, b'{"measured_distance_m":null}\n')
        for name in ('config/hardware/runtime_board.json', 'config/hardware/steering_zero.json',
                     'config/hardware/credentials.json', 'config/perception/measured_installation.json',
                     'config/tests/random_case.json', 'src/arena_description/config/unselected.yaml'):
            self.put(name, b'{"not_to_copy":true}\n')
        self.put(candidate.TEMPLATE, b'# original template\n')
        files = candidate.collect(self.root)
        self.assertEqual(set(files), set(candidate.PUBLIC_RESOURCES) | {'handover/runtime_profile.template.yaml'})
        for name in candidate.PUBLIC_RESOURCES:
            self.assertEqual(files[name], b'{"measured_distance_m":null}\n')
        report = candidate.inventory(files)
        self.assertTrue(all(row['automatically_selected'] is False for row in report['public_resources'].values()))

    def test_synthetic_zip_matches_manifest(self):
        files = {n:b'# synthetic fixture, not deployed\n' for n in candidate.REQUIRED}
        target = self.root/'synthetic.zip'
        candidate.write_bundle(files, target)
        with zipfile.ZipFile(target) as z:
            report = json.loads(z.read('source_manifest.json'))
            self.assertTrue(report['source_complete'])
            for n, digest in report['files'].items(): self.assertEqual(candidate.sha(z.read(n)), digest)
        self.assertFalse(report['device_io'])


if __name__ == '__main__': unittest.main()
