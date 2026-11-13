"""단일 축 PI의 Python/C++ 산술 대조. 실물 게인·구동/제동 성능 검사가 아니다."""
import json
import math
from pathlib import Path
import random
import shutil
import subprocess

import pytest

from arena_vehicle_interface.mcu_speed_loop import SpeedPI

ROOT = Path(__file__).resolve().parents[1]
INCLUDE = ROOT / 'src/arena_vehicle_interface/portable'


@pytest.fixture(scope='module', params=['sanitizers', 'optimized'])
def driver(request, tmp_path_factory):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ required')
    output = tmp_path_factory.mktemp('speed-pi') / 'driver'
    flags = ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-ffp-contract=off',
             '-fno-exceptions', '-fno-rtti']
    flags += (['-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer']
              if request.param == 'sanitizers' else ['-Os'])
    result = subprocess.run([compiler, *flags, f'-I{INCLUDE}',
                             str(ROOT / 'scripts/speed_pi_cpp_driver.cpp'), '-o', str(output)],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    return output


def run(driver, samples, gains=(.6, 1.2, .05)):
    lines = [' '.join(map(repr, gains))]
    lines += [' '.join(map(repr, (*s[:3], int(s[3])))) for s in samples]
    result = subprocess.run([str(driver)], input='\n'.join(lines) + '\n',
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0 and not result.stderr, (result.stdout, result.stderr)
    rows = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(rows) == len(samples)
    return rows


def compare(driver, samples, gains=(.6, 1.2, .05)):
    pi = SpeedPI(*gains)
    rows = run(driver, samples, gains)
    for row, (target, measured, dt, enabled) in zip(rows, samples):
        expected = pi.update_mean(target, measured, dt, enabled=enabled)
        assert row['effort'] == expected.effort
        assert row['integral'] == pi.integral
        assert row['brake_requested'] is expected.brake_requested
        assert row['valid'] is True
        assert math.isfinite(row['effort']) and -1 <= row['effort'] <= 1
        assert -1 <= row['integral'] <= 1
    return rows


@pytest.mark.parametrize('seed', range(8))
def test_sequences_equal_reference(driver, seed):
    rng = random.Random(seed)
    samples = [(rng.uniform(0.01, 6), rng.uniform(-1, 7), rng.choice([.001, .005, .01, .05]),
                rng.random() > .03) for _ in range(500)]
    samples[::61] = [(0., .1, .005, True)] * len(samples[::61])
    compare(driver, samples)


def test_saturation_unwind_and_resets(driver):
    samples = ([(1., .8, .05, True)] * 50 + [(5., 0., .005, True)] * 100 +
               [(.2, 4., .005, True)] * 100 + [(1., 1.3, .05, True)] * 50 +
               [(0., 1., .005, True), (1., .5, .005, True), (1., .5, .005, False),
                (1., .5, .005, True), (1., .5, .051, True), (1., .5, .005, True)])
    rows = compare(driver, samples)
    assert rows[49]['integral'] > 0
    assert rows[50]['integral'] == rows[149]['integral']  # 포화 쪽 누적 금지
    assert rows[150]['effort'] == -1 and rows[150]['brake_requested']
    assert rows[300]['integral'] == rows[302]['integral'] == rows[304]['integral'] == 0
    assert rows[301]['effort'] == rows[303]['effort'] == rows[305]['effort'] == .303


@pytest.mark.parametrize('index', range(3))
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_input_clears_integral(driver, index, value):
    bad = [1., .5, .005, True]
    bad[index] = value
    rows = compare(driver, [(1., .5, .005, True), tuple(bad), (1., .5, .005, True)])
    assert rows[1]['status'] == 4 and rows[1]['integral'] == 0 and rows[1]['effort'] == 0
    assert rows[0]['effort'] == rows[2]['effort']


@pytest.mark.parametrize('sample,status', [
    ((-1., 0., .005, True), 4), ((1., 0., 0., True), 4), ((1., 0., -.01, True), 4),
    ((1., 0., math.nextafter(.05, math.inf), True), 4),
    ((1., 0., .05, True), 0), ((0., 0., .005, True), 2),
    ((-0., 0., .005, True), 2), ((1., 0., .005, False), 1),
    ((1e308, -1e308, .005, True), 5),
])
def test_edge_status(driver, sample, status):
    assert compare(driver, [sample])[0]['status'] == status


@pytest.mark.parametrize('gains', [(1e308, 1.2, .05), (.6, 1e308, .05)])
def test_finite_gain_overflow_inhibits(driver, gains):
    row = compare(driver, [(2., 0., .005, True)], gains)[0]
    assert row['effort'] == 0 and row['brake_requested'] and row['status'] == 5


@pytest.mark.parametrize('index', range(3))
@pytest.mark.parametrize('value', [0., -1., float('inf'), float('nan')])
def test_invalid_config_stays_disabled(driver, index, value):
    gains = [.6, 1.2, .05]
    gains[index] = value
    with pytest.raises(ValueError):
        SpeedPI(*gains)
    rows = run(driver, [(1., 0., .005, True)] * 2, gains)
    assert all(row == dict(effort=0., brake_requested=True, integral=0., valid=False, status=3)
               for row in rows)


def test_normal_python_matches_prefault_equation():
    # 수정 전 speed_pi_numeric_v1/source_snapshot.tar.gz 수식의 정상 범위 독립 대조.
    rng = random.Random(433)
    pi = SpeedPI()
    old_integral = 0.
    for _ in range(4000):
        target, measured, dt = rng.uniform(.01, 6), rng.uniform(-2, 8), rng.uniform(.0001, .05)
        error = target - measured
        candidate = old_integral + 1.2 * error * dt
        raw = .6 * error + candidate
        if -1 <= raw <= 1 or (raw > 1 and error < 0) or (raw < -1 and error > 0):
            old_integral = max(-1., min(1., candidate))
        old_effort = max(-1., min(1., .6 * error + old_integral))
        result = pi.update_mean(target, measured, dt, enabled=True)
        assert result.effort == old_effort and pi.integral == old_integral
        assert result.brake_requested == (old_effort < 0)


@pytest.mark.parametrize('flag', ['-ffast-math', '-ffinite-math-only'])
def test_unsafe_fp_compile_mode_rejected(tmp_path, flag):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ required')
    result = subprocess.run([compiler, '-std=c++17', flag, f'-I{INCLUDE}', '-c',
                             str(ROOT / 'scripts/speed_pi_cpp_driver.cpp'), '-o', str(tmp_path / 'bad.o')],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode != 0 and 'SpeedPI requires nonfinite checks' in result.stderr
