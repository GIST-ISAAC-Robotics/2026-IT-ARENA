"""staged 소스만 빌드하는 합성 검사. 실물 교정/출력/성능 시험이 아니다."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import random
import shutil
import subprocess
import sys

import pytest

A = Path(__file__).resolve().parents[1]
ROOT = A.parents[3]
STAGED = A / 'staged'
PORTABLE = Path('src/arena_vehicle_interface/portable')
MODULE = Path('src/arena_vehicle_interface/arena_vehicle_interface/mcu_speed_loop.py')
sys.path.insert(0, str(ROOT / 'src/arena_vehicle_interface'))


def load_pi(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.SpeedPI


OriginalPI = load_pi('r381_original_pi', ROOT / MODULE)
CandidatePI = load_pi('r381_candidate_pi', STAGED / MODULE)


@pytest.fixture(scope='session', autouse=True)
def unchanged_sources():
    expected = json.loads((A / 'tests/source_manifest.json').read_text(encoding='utf-8-sig'))
    prepared = json.loads((A / 'tests/prepared_manifest.json').read_text())['files']
    def check():
        for row in expected:
            assert hashlib.sha256((ROOT / row['path']).read_bytes()).hexdigest() == row['sha256'], row['path']
        for name, digest in prepared.items():
            assert hashlib.sha256((A / name).read_bytes()).hexdigest() == digest, name
    check()
    yield
    check()


@pytest.fixture(scope='session')
def drivers(tmp_path_factory):
    compiler = shutil.which('g++')
    assert compiler, 'g++ required; missing compiler is not a passed/skip result'
    build = tmp_path_factory.mktemp('r381-forward')
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-Os',
             '-ffp-contract=off', '-fno-exceptions', '-fno-rtti']
    sources = {
        'original': ROOT / 'scripts/speed_pi_cpp_driver.cpp',
        'default': ROOT / 'scripts/speed_pi_cpp_driver.cpp',
        'limits': A / 'tests/speed_pi_limits_driver.cpp',
        'runtime': A / 'tests/forward_runtime_cases.cpp',
    }
    binaries = {}
    for name, source in sources.items():
        includes = [] if name == 'original' else [f'-I{STAGED / PORTABLE}']
        includes += [f'-I{ROOT / PORTABLE}', f'-I{ROOT / "src/arena_vehicle_interface/firmware_smoke/main"}']
        output = build / name
        result = subprocess.run([compiler, *flags, *includes, str(source), '-o', str(output)],
                                capture_output=True, text=True, timeout=90)
        assert result.returncode == 0, result.stderr
        binaries[name] = output
    return binaries


def run(driver, samples, settings=(.6, 1.2, .05)):
    lines = [' '.join(map(repr, settings))]
    lines += [' '.join(map(repr, (*sample[:3], int(sample[3])))) for sample in samples]
    result = subprocess.run([str(driver)], input='\n'.join(lines) + '\n',
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 0 and not result.stderr, (result.stdout, result.stderr)
    rows = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(rows) == len(samples)
    return result.stdout, rows


def same_float(a, b):
    # -0도 포함해 binary64 값의 정확 일치를 요구한다.
    assert float(a).hex() == float(b).hex()


def compare_limits(driver, samples, limits):
    pi = CandidatePI(output_min=limits[0], output_max=limits[1])
    _, rows = run(driver, samples, (.6, 1.2, .05, *limits))
    for row, (target, measured, dt, enabled) in zip(rows, samples):
        intent = pi.update_mean(target, measured, dt, enabled=enabled)
        same_float(row['effort'], intent.effort)
        same_float(row['integral'], pi.integral)
        assert row['brake_requested'] is intent.brake_requested
        assert row['valid'] and limits[0] <= row['effort'] <= limits[1]
        assert limits[0] <= row['integral'] <= limits[1]
    return rows


def test_default_full_output_unchanged(drivers):
    rng = random.Random(381)
    samples = [(rng.uniform(.01, 6), rng.uniform(-3, 8), rng.choice([.001, .005, .01, .05]),
                rng.random() > .04) for _ in range(2000)]
    samples += [(0., .5, .005, True), (-0., .1, .005, True), (1., 0., 0., True),
                (1., 0., math.nextafter(.05, math.inf), True), (-1., 0., .005, True),
                (1e308, -1e308, .005, True)]
    for index in range(3):
        for value in (math.nan, math.inf, -math.inf):
            bad = [.5, .1, .005, True]; bad[index] = value
            samples.append(tuple(bad))
    old_text, rows = run(drivers['original'], samples)
    new_text, _ = run(drivers['default'], samples)
    explicit_text, _ = run(drivers['limits'], samples, (.6, 1.2, .05, -1., 1.))
    assert old_text == new_text == explicit_text  # effort/brake/integral/valid/status 원문 전체
    old, new = OriginalPI(), CandidatePI()
    for row, (target, measured, dt, enabled) in zip(rows, samples):
        a = old.update_mean(target, measured, dt, enabled=enabled)
        b = new.update_mean(target, measured, dt, enabled=enabled)
        same_float(a.effort, b.effort)
        same_float(old.integral, new.integral)
        assert a.brake_requested is b.brake_requested is row['brake_requested']
        same_float(row['effort'], b.effort)
        same_float(row['integral'], new.integral)


@pytest.mark.parametrize('dt', [.001, .005, .01, .05])
@pytest.mark.parametrize('limits', [(0., .04), (0., .4), (0., 1.), (-.4, .7)])
def test_limits_direction_and_dt(drivers, dt, limits):
    samples = ([(.5, 0., dt, True)] * 20 + [(.5, -.5, dt, True)] * 20 +
               [(.5, 3., dt, True)] * 20 + [(.5, .4, dt, True)] * 20 +
               [(0., .2, dt, True), (.5, .2, dt, True), (.5, .2, dt, False)])
    rows = compare_limits(drivers['limits'], samples, limits)
    assert all(row['effort'] > 0 for row in rows[20:40])  # 역회전 피드백에 양수 전진 반응
    if limits[0] == 0:
        assert all(row['effort'] == 0 and row['status'] == 0 and not row['brake_requested']
                   for row in rows[40:60])
    assert rows[-3]['integral'] == rows[-1]['integral'] == 0


def test_upper_lower_saturation_and_release(drivers):
    samples = ([(1., .9, .05, True)] * 40 + [(2., 0., .05, True)] * 100 +
               [(1., 1.1, .05, True)] + [(0., 0., .005, True)] +
               [(.5, 3., .005, True)] * 100 + [(.5, 0., .005, True)])
    rows = compare_limits(drivers['limits'], samples, (0., .4))
    assert rows[39]['integral'] > 0
    assert rows[40]['integral'] == rows[139]['integral'] == rows[39]['integral']
    assert all(row['effort'] == .4 for row in rows[40:140])
    assert 0 < rows[140]['effort'] < .4 and rows[140]['integral'] < rows[139]['integral']
    assert all(row['effort'] == row['integral'] == 0 and row['status'] == 0 for row in rows[142:242])
    assert rows[242]['effort'] == .303  # 하한에서 탈출할 때 음수 적분 잔류 없음


@pytest.mark.parametrize('limits', [(0., 0.), (.1, .4), (-.4, -.1), (-1.1, 1.), (0., 1.1),
                                   (math.nan, .4), (0., math.nan), (-math.inf, 1.), (0., math.inf)])
def test_invalid_limits(drivers, limits):
    with pytest.raises(ValueError):
        CandidatePI(output_min=limits[0], output_max=limits[1])
    _, rows = run(drivers['limits'], [(.5, 0., .005, True)] * 2, (.6, 1.2, .05, *limits))
    assert all(row == dict(effort=0., brake_requested=True, integral=0., valid=False, status=3)
               for row in rows)


def test_invalid_inputs_and_reset(drivers):
    valid = (.5, 0., .005, True)
    invalid = [(-1., 0., .005, True), (.5, 0., 0., True), (.5, 0., -.01, True),
               (.5, 0., math.nextafter(.05, math.inf), True), (1e308, -1e308, .005, True)]
    for index in range(3):
        for value in (math.nan, math.inf, -math.inf):
            bad = list(valid); bad[index] = value; invalid.append(tuple(bad))
    for bad in invalid:
        rows = compare_limits(drivers['limits'], [valid, bad, valid], (0., .4))
        assert rows[1]['effort'] == rows[1]['integral'] == 0 and rows[1]['status'] in (4, 5)
        assert rows[0] == rows[2]
    for bad_enabled in (1, 0, None, 'true'):
        pi = CandidatePI(output_min=0., output_max=.4)
        pi.update_mean(*valid[:3], enabled=True)
        assert pi.update_mean(*valid[:3], enabled=bad_enabled).effort == pi.integral == 0


def test_existing_receiver_and_forward_mapping(drivers):
    result = subprocess.run([str(drivers['runtime'])], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0 and not result.stderr, (result.stdout, result.stderr)
    assert 'PASS' in result.stdout
