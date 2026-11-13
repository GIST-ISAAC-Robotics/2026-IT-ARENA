"""실물 UART/시간 검증이 아닌 portable 순서기의 합성 요청/응답 검사."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import pytest

A=Path(__file__).resolve().parents[1]
ROOT=A.parents[3]


@pytest.fixture(scope='session',autouse=True)
def frozen():
    manifest=json.loads((A/'manifest.json').read_text(encoding='utf-8'))
    def check():
        for name,digest in manifest['sources'].items():
            assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
        for name,digest in manifest['prepared'].items():
            assert hashlib.sha256((A/name).read_bytes()).hexdigest()==digest,name
    check()
    yield
    check()


@pytest.fixture(scope='session')
def driver(tmp_path_factory):
    compiler=shutil.which('g++')
    assert compiler,'g++ required; do not count missing compiler as skip/pass'
    output=tmp_path_factory.mktemp('servo-r384')/'driver'
    result=subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-Os',
        '-ffp-contract=off','-fno-exceptions','-fno-rtti',
        f'-I{A/"staged/src/arena_vehicle_interface/portable"}',
        f'-I{ROOT/"src/arena_vehicle_interface/portable"}',
        str(A/'tests/servo_runtime_cases.cpp'),'-o',str(output)],capture_output=True,text=True,timeout=90)
    assert result.returncode==0,result.stderr
    return output


CASES=['basic','latest','partial','freshness','stopped_goal','backpressure_timeout']
CASES += [f'{kind}{stage}' for kind in ('stop','generation','timeout') for stage in range(3)]
CASES += ['unsent0','unsent1']
CASES += ['invalid_'+kind for kind in ('profile','default','speed','range','mode','position','engage_position','status','goal','clock')]


@pytest.mark.parametrize('case',CASES)
def test_servo_sequence(driver,case):
    result=subprocess.run([str(driver),case],capture_output=True,text=True,timeout=10)
    assert result.returncode==0 and not result.stderr,(result.stdout,result.stderr)
    assert result.stdout.strip()==case+': PASS'
