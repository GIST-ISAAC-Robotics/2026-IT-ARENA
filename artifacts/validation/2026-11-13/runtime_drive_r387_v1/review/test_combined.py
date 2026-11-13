"""실제 portable 경계의 합성 결합. main/ESP-IDF/실물 시각은 실행하지 않는다."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import pytest

A=Path(__file__).resolve().parent
ROOT=A.parents[4]
CASES=['default','engagement','arm_no_center','stop_late_ack','expired_late_ack',
       'control_gap','negative_feedback','servo_fault_recovery','profile_gains','profile_geometry','quantization']


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
    assert compiler,'g++ required; missing compiler is failure, not skip/pass'
    output=tmp_path_factory.mktemp('runtime-drive-review')/'driver'
    flags=['-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-Os',
           '-ffp-contract=off','-fno-exceptions','-fno-rtti']
    includes=[ROOT/'src/arena_vehicle_interface'/path for path in
              ('portable','firmware_runtime_drive/main','firmware_runtime_preview/main')]
    result=subprocess.run([compiler,*flags,*[f'-I{p}' for p in includes],
        str(A/'combined_cases.cpp'),'-o',str(output)],capture_output=True,text=True,timeout=90)
    assert result.returncode==0,result.stderr
    return output


@pytest.mark.parametrize('case',CASES)
def test_runtime_connections(driver,case):
    result=subprocess.run([str(driver),case],capture_output=True,text=True,timeout=15)
    assert result.returncode==0 and not result.stderr,(result.stdout,result.stderr)
    assert result.stdout.strip()==case+': PASS'
